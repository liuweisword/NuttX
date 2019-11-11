#include <stdlib.h>
#include <string.h>

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_master.h>

#include "chip/ar8020_memorymap.h"
#include "chip/ar_config.h"
#include "chip/ar_define.h"
#include "up_arch.h"
#include "ar_i2c.h"
#include "ar_uart.h"

#if !defined(CONFIG_AR_I2CTIMEOSEC) && !defined(CONFIG_AR_I2CTIMEOMS)
#define CONFIG_AR_I2CTIMEOSEC 0
#define CONFIG_AR_I2CTIMEOMS 500 /* Default is 500 milliseconds */
#warning "Using Default 500 Ms Timeout"
#elif !defined(CONFIG_AR_I2CTIMEOSEC)
#define CONFIG_AR_I2CTIMEOSEC 0 /* User provided milliseconds */
#elif !defined(CONFIG_AR_I2CTIMEOMS)
#define CONFIG_AR_I2CTIMEOMS 0 /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_AR_I2CTIMEOTICKS
#define CONFIG_AR_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_AR_I2CTIMEOSEC) + MSEC2TICK(CONFIG_AR_I2CTIMEOMS))
#endif

#ifndef CONFIG_AR_I2C_DYNTIMEO_STARTSTOP
#define CONFIG_AR_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_AR_I2CTIMEOTICKS)
#endif

#define AR_I2C_BUFFER_LENGTH 128

enum ar_intstate_e
{
    INTSTATE_IDLE = 0, /* No I2C activity */
    INTSTATE_WAITING,  /* Waiting for completion of interrupt activity */
    INTSTATE_DONE,     /* Interrupt activity complete */
};

struct ar_i2c_config_s
{
    uint32_t base;   /* I2C base address */
    uint32_t ev_irq; /* Event IRQ */
};

typedef enum
{
    AR_I2C_OP_WRITE = 0,
    AR_I2C_OP_READ = 1

} ENUM_AR_I2C_OP;

typedef struct
{
    uint32_t tx_len;     // 发送的总长度
    uint32_t tx_alr_len; // 已经发送的长度
    uint8_t *txbuf;      // 发送的buffer
    uint32_t rx_len;     // 接受的总长度
    uint32_t rx_alr_len; // 已经接收的长度
    uint8_t *rxbuf;      // 接收的buffer

} AR_I2C_INT_DATA;

/* I2C Device Private Data */
struct ar_i2c_priv_s
{
    const struct ar_i2c_config_s *config; /* Port configuration */
    int refs;                             /* Referernce count */
    sem_t sem_excl;                       /* Mutual exclusion semaphore */
    sem_t sem_isr;                        /* Interrupt wait semaphore */
    volatile uint8_t intstate;            /* Interrupt handshake (see enum ar_intstate_e) */

    AR_I2C_INT_DATA i2c_int_data;

    uint16_t addr; /* Slave address (7- or 10-bit) */

    uint32_t frequency; /* I2C frequency */

    struct i2c_msg_s *msgv; /* message info */

    uint8_t msgc; /* Message count */

    uint32_t status;

    uint32_t abort_source;
};

/* is2C Device, Instance */

struct ar_i2c_inst_s
{
    const struct i2c_ops_s *ops; /* Standard I2C operations */
    struct ar_i2c_priv_s *priv;  /* Common driver private data structure */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/
static inline uint32_t ar_i2c_getreg32(FAR struct ar_i2c_priv_s *priv, uint8_t offset);
static inline void ar_i2c_putreg32(FAR struct ar_i2c_priv_s *priv, uint8_t offset,
                                   uint32_t value);
static inline void ar_i2c_modifyreg32(FAR struct ar_i2c_priv_s *priv,
                                      uint8_t offset, uint32_t clearbits,
                                      uint32_t setbits);
static inline void ar_i2c_sem_wait(FAR struct i2c_master_s *dev);
static inline int ar_i2c_sem_waitdone(FAR struct ar_i2c_priv_s *priv);
static inline void ar_i2c_sem_waitstop(FAR struct ar_i2c_priv_s *priv);
static inline void ar_i2c_sem_post(FAR struct i2c_master_s *dev);
static inline void ar_i2c_sem_init(FAR struct i2c_master_s *dev);
static inline void ar_i2c_sem_destroy(FAR struct i2c_master_s *dev);

static int ar_i2c_init(FAR struct ar_i2c_priv_s *priv);
static int ar_i2c_deinit(FAR struct ar_i2c_priv_s *priv);

static int ar_i2c_process(FAR struct i2c_master_s *dev, FAR struct i2c_msg_s *msgs, int count);
static int ar_i2c_transfer(FAR struct i2c_master_s *dev, FAR struct i2c_msg_s *msgs, int count);
static int ar_i2c_reset(FAR struct i2c_master_s *dev);

static inline void ar_i2c_clearinterrupts(struct ar_i2c_priv_s *priv);
static inline void ar_i2c_clear_rxunder_interrupts(struct ar_i2c_priv_s *priv);

static inline void ar_i2c_enableinterrupts(struct ar_i2c_priv_s *priv);

static inline void ar_i2c_disable_all_interrupts(struct ar_i2c_priv_s *priv);
static inline void ar_i2c_disable_txempty_interrupts(struct ar_i2c_priv_s *priv);

static inline void ar_i2c_sendstart(FAR struct ar_i2c_priv_s *priv);
static inline void ar_i2c_sendstop(FAR struct ar_i2c_priv_s *priv);

static inline uint8_t ar_i2c_set_rxtl(FAR struct ar_i2c_priv_s *priv, uint32_t tl);
static uint8_t ar_i2c_set_txtl(struct ar_i2c_priv_s *priv, uint32_t tl);

static inline void ar_i2c_clear_interrupts(struct ar_i2c_priv_s *priv, uint8_t offset);

static inline void ar_i2c_load_fifo_write_data(struct ar_i2c_priv_s *priv);
static inline void ar_i2c_fifo_read_data(FAR struct ar_i2c_priv_s *priv);

static const struct ar_i2c_config_s ar_i2c1_config =
    {
        .base = AR_I2C0_BASE,
        .ev_irq = AR_IRQ_I2C0,
};

static const struct ar_i2c_config_s ar_i2c2_config =
    {
        .base = AR_I2C1_BASE,
        .ev_irq = AR_IRQ_I2C1,
};

static const struct ar_i2c_config_s ar_i2c3_config =
    {
        .base = AR_I2C2_BASE,
        .ev_irq = AR_IRQ_I2C2,
};

static const struct ar_i2c_config_s ar_i2c4_config =
    {
        .base = AR_I2C3_BASE,
        .ev_irq = AR_IRQ_I2C3,
};

static struct ar_i2c_priv_s ar_i2c1_priv =
    {
        .config = &ar_i2c1_config,
        .refs = 0,
        .intstate = INTSTATE_IDLE,
        .msgc = 0,
        .msgv = NULL,
};

static struct ar_i2c_priv_s ar_i2c2_priv =
    {
        .config = &ar_i2c2_config,
        .refs = 0,
        .intstate = INTSTATE_IDLE,
        .msgc = 0,
        .msgv = NULL,
};

static struct ar_i2c_priv_s ar_i2c3_priv =
    {
        .config = &ar_i2c3_config,
        .refs = 0,
        .intstate = INTSTATE_IDLE,
        .msgc = 0,
        .msgv = NULL,
};

static struct ar_i2c_priv_s ar_i2c4_priv =
    {
        .config = &ar_i2c4_config,
        .refs = 0,
        .intstate = INTSTATE_IDLE,
        .msgc = 0,
        .msgv = NULL,
};

static const struct i2c_ops_s ar_i2c_ops =
    {
        .transfer = ar_i2c_transfer,
#ifdef CONFIG_I2C_RESET
        .reset = ar_i2c_reset
#endif
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int ar_i2c_deinit(FAR struct ar_i2c_priv_s *priv)
{
    up_disable_irq(priv->config->ev_irq);

    irq_detach(priv->config->ev_irq);

    return OK;
}

#ifdef CONFIG_I2C_RESET
static int ar_i2c_reset(FAR struct i2c_master_s *dev)
{
    i2cinfo("fail i2c, reset \r\n");
    FAR struct ar_i2c_priv_s *priv = ((struct ar_i2c_inst_s *)dev)->priv;

    /* Our caller must own a ref */
    ASSERT(priv->refs > 0);

    /* Lock out other clients */

    ar_i2c_sem_wait(dev);

    /* De-init the port */
    ar_i2c_deinit(priv);

     /* Re-init the port */
    ar_i2c_init(priv);

    /* Re-init the port */
    ar_i2c_sem_post(dev);

    return OK;
}
#endif

static inline uint32_t ar_i2c_getreg32(FAR struct ar_i2c_priv_s *priv,
                                       uint8_t offset)
{
    return getreg32(priv->config->base + offset);
}

static inline void ar_i2c_putreg32(FAR struct ar_i2c_priv_s *priv,
                                   uint8_t offset, uint32_t value)
{
    putreg32(value, priv->config->base + offset);
}

static inline void ar_i2c_modifyreg32(FAR struct ar_i2c_priv_s *priv,
                                      uint8_t offset, uint32_t clearbits,
                                      uint32_t setbits)
{
    modifyreg32(priv->config->base + offset, clearbits, setbits);
}

static inline void ar_i2c_sem_wait(FAR struct i2c_master_s *dev)
{
    while (sem_wait(&((struct ar_i2c_inst_s *)dev)->priv->sem_excl) != 0)
    {
        ASSERT(errno == EINTR);
    }
}

static useconds_t ar_i2c_tousecs(FAR struct ar_i2c_priv_s *priv)
{
    size_t bytecount = 0;

    bytecount += (priv->i2c_int_data.rx_len + priv->i2c_int_data.tx_len);

    return (useconds_t)(CONFIG_AR_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}

static inline int ar_i2c_sem_waitdone(FAR struct ar_i2c_priv_s *priv)
{
    struct timespec abstime;
    irqstate_t flags;
    int ret;

    // i2cinfo("ar_i2c_sem_waitdone---------debug\r\n");
    flags = enter_critical_section();

    /* The TXIE and RXIE interrupts are enabled initially in ar_i2c_process.
    * The remainder of the interrupts, including error-related, are enabled here.
    */

    /* Signal the interrupt handler that we are waiting */

    // priv->intstate = INTSTATE_WAITING;
    do
    {
        /* Get the current time */

        (void)clock_gettime(CLOCK_REALTIME, &abstime);

        /* Add a value proportional to the number of bytes in the transfer */
        // // Todo:     
        ar_i2c_tousecs(priv);

        abstime.tv_nsec += 10000 * ar_i2c_tousecs(priv);
        
        // abstime.tv_sec++;

        if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
            abstime.tv_sec++;
            abstime.tv_nsec -=  10000 * ar_i2c_tousecs(priv);
        }

        // i2cinfo("abstime=%d  nsecond = %ld\r\n",abstime.tv_sec, abstime.tv_nsec);
        /* Wait until either the transfer is complete or the timeout expires */

        ret = sem_timedwait(&priv->sem_isr, &abstime);
        // ret = sem_wait(&priv->sem_isr);

        if (ret != OK && errno != EINTR)
        {
            /* Break out of the loop on irrecoverable errors.  This would
             * include timeouts and mystery errors reported by sem_timedwait.
             * NOTE that we try again if we are awakened by a signal (EINTR).
             */
            i2cerr("ret != OK && errno = %d\r\n", errno);
            break;
        }
    } while (priv->intstate != INTSTATE_DONE);

    /* Set the interrupt state back to IDLE */

    priv->intstate = INTSTATE_IDLE;

    /* Disable I2C interrupts */

    ar_i2c_disable_all_interrupts(priv);

    leave_critical_section(flags);

    return ret;
}

static inline void ar_i2c_sem_waitstop(FAR struct ar_i2c_priv_s *priv)
{
    uint32_t start;
    uint32_t elapsed;
    uint32_t timeout;
    uint32_t cr;

    /* Select a timeout */

#ifdef CONFIG_AR_I2C_DYNTIMEO
    timeout = USEC2TICK(CONFIG_AR_I2C_DYNTIMEO_STARTSTOP);
#else
    timeout = CONFIG_AR_I2CTIMEOTICKS;
#endif

    /* Wait as stop might still be in progress */

    start = clock_systimer();
    do
    {
        /* Calculate the elapsed time */

        elapsed = clock_systimer() - start;

        /* Check for status condition */

        cr = ar_i2c_getreg32(priv, I2C_IC_STATUS);
        if ((cr & IC_STATUS_MST_ACTIVITY) == IC_STATUS_MST_ACTIVITY_IDLE)
        {
            return;
        }

    }

    /* Loop until the stop is complete or a timeout occurs. */

    while (elapsed < timeout);

    /* If we get here then a timeout occurred with the STOP condition
    * still pending.
    */
}

static inline void ar_i2c_sem_post(FAR struct i2c_master_s *dev)
{
    sem_post(&((struct ar_i2c_inst_s *)dev)->priv->sem_excl);
}

static inline void ar_i2c_sem_init(FAR struct i2c_master_s *dev)
{
    sem_init(&((struct ar_i2c_inst_s *)dev)->priv->sem_excl, 0, 1);

    sem_init(&((struct ar_i2c_inst_s *)dev)->priv->sem_isr, 0, 0);
    sem_setprotocol(&((struct ar_i2c_inst_s *)dev)->priv->sem_isr, SEM_PRIO_NONE);
}

static inline void ar_i2c_sem_destroy(FAR struct i2c_master_s *dev)
{
    sem_destroy(&((struct ar_i2c_inst_s *)dev)->priv->sem_excl);
    sem_destroy(&((struct ar_i2c_inst_s *)dev)->priv->sem_isr);
}

static inline void ar_i2c_set_7bit_address(FAR struct ar_i2c_priv_s *priv)
{
    ar_i2c_putreg32(priv, I2C_IC_TAR, priv->addr & 0x7F);
}

static uint32_t ar_i2c_master_getrxfifolength(struct ar_i2c_priv_s *priv)
{
    return ar_i2c_getreg32(priv, I2C_IC_RXFLR);
}

static uint32_t ar_i2c_master_gettxfifolength(struct ar_i2c_priv_s *priv)
{
    return ar_i2c_getreg32(priv, I2C_IC_TXFLR);
}

static void ar_i2c_disable(struct ar_i2c_priv_s *priv)
{
    ar_i2c_modifyreg32(priv, I2C_IC_ENABLE, IC_ENABLE_ENABLE, 0);
}

static void ar_i2c_enable(struct ar_i2c_priv_s *priv)
{
    ar_i2c_modifyreg32(priv, I2C_IC_ENABLE, 0, IC_ENABLE_ENABLE);
}

static void ar_i2c_setclock(FAR struct ar_i2c_priv_s *priv, uint32_t frequency)
{
    uint32_t pe;
    uint32_t data_ctrl;

    /* I2C peripheral must be disabled to update clocking configuration */
    pe = (ar_i2c_getreg32(priv, I2C_IC_ENABLE) & IC_ENABLE_ENABLE);
    if (pe)
    {
        ar_i2c_disable(priv);
    }

    if (frequency == 100000) // stardard speed
    {
        data_ctrl = ar_i2c_getreg32(priv, I2C_IC_CON);
        data_ctrl |= IC_CON_SPEED_MASK;
        data_ctrl &= 0xFB;
        // data_ctrl |= IC_CON_SPEED_STANDARD_MODE;
        ar_i2c_putreg32(priv, I2C_IC_CON, data_ctrl);

        ar_i2c_putreg32(priv, I2C_IC_FS_SPKLEN, 1);
        // set high period of SCL 4000 * 2 ns     100MHz = 10ns   8000ns = 800cnt
        ar_i2c_putreg32(priv, I2C_IC_SS_SCL_HCNT, 650);

        // set low period of SCL 4700 * 2 ns
        ar_i2c_putreg32(priv, I2C_IC_SS_SCL_LCNT, 650);
    }
    else if (frequency == 400000) // fast speed
    {
        data_ctrl = ar_i2c_getreg32(priv, I2C_IC_CON);
        data_ctrl |= IC_CON_SPEED_MASK;
        data_ctrl &= 0xFD;
        ar_i2c_putreg32(priv, I2C_IC_CON, data_ctrl);

        ar_i2c_putreg32(priv, I2C_IC_FS_SPKLEN, 1);
        // set high period of SCL 600 * 2 ns
        ar_i2c_putreg32(priv, I2C_IC_FS_SCL_HCNT, 160);

        // set low period of SCL 1300 * 2 ns
        ar_i2c_putreg32(priv, I2C_IC_FS_SCL_LCNT, 160);
    }
    else if (frequency == 1000000) // high mode
    {
        data_ctrl = ar_i2c_getreg32(priv, I2C_IC_CON);
        data_ctrl |= IC_CON_SPEED_MASK;
        ar_i2c_putreg32(priv, I2C_IC_CON, data_ctrl);

        ar_i2c_putreg32(priv, I2C_IC_HS_SPKLEN, 1);
        // set high period of SCL 160 * 2 ns
        ar_i2c_putreg32(priv, I2C_IC_HS_SCL_HCNT, 65);

        // set low period of SCL 320 * 2 ns
        ar_i2c_putreg32(priv, I2C_IC_HS_SCL_LCNT, 65);
    }

    if (pe)
    {
        ar_i2c_enable(priv);
    }
}

static inline void i2c_write_byte(struct ar_i2c_priv_s *priv, uint8_t data)
{
    ar_i2c_putreg32(priv, I2C_IC_DATA_CMD, data & 0xFF);
}

static inline void i2c_read_launch(struct ar_i2c_priv_s *priv)
{
    ar_i2c_putreg32(priv, I2C_IC_DATA_CMD, IC_DATA_CMD_CMD);
}

static inline void ar_i2c_load_fifo_write_data(struct ar_i2c_priv_s *priv)
{
    int32_t fifolength = ar_i2c_master_gettxfifolength(priv);

    uint32_t cnt = (I2C_TX_FIFO_BUFFER_DEPTH - fifolength);

    uint32_t remain_tx_len = priv->i2c_int_data.tx_len - priv->i2c_int_data.tx_alr_len;

    if (remain_tx_len <= cnt)
    {
        ar_i2c_set_txtl(priv, 0);
    }
    else
    {
        ar_i2c_set_txtl(priv, IC_TX_TL_DEF_VALUE);
    }

    cnt = remain_tx_len < cnt ? remain_tx_len : cnt;

    for (uint16_t i = 0; i < cnt; i++)
    {
        unsigned int data = priv->i2c_int_data.txbuf[priv->i2c_int_data.tx_alr_len++];

        i2c_write_byte(priv, data);
    }

    // ar_i2c_enableinterrupts(priv);
    ar_i2c_modifyreg32(priv, I2C_IC_INTR_MASK, IC_INTR_M_MASK, IC_INTR_R_TX_EMPTY | IC_INTR_R_TX_ABRT | IC_INTR_R_STOP_DET);
}

static inline void ar_i2c_fifo_read_data(FAR struct ar_i2c_priv_s *priv)
{
    int32_t fifolength = ar_i2c_master_gettxfifolength(priv); // read tx 1

    uint32_t cnt = (I2C_RX_FIFO_BUFFER_DEPTH - fifolength); // 7

    uint32_t remain_rx_len = priv->i2c_int_data.rx_len - priv->i2c_int_data.rx_alr_len; //1

    cnt = remain_rx_len < cnt ? remain_rx_len : cnt; // cnt = 1



    if (remain_rx_len <= cnt)
    {
        ar_i2c_set_rxtl(priv, cnt - 1);//0
    }
    else
    {
        ar_i2c_set_rxtl(priv, IC_RX_TL_DEF_VALUE);
    }

    for (uint16_t i = 0; i < cnt; i++)
    {
        i2c_read_launch(priv);
    }

    ar_i2c_modifyreg32(priv, I2C_IC_INTR_MASK, IC_INTR_M_MASK, IC_INTR_R_RX_FULL | IC_INTR_R_TX_ABRT);
}

static inline uint8_t ar_i2c_set_rxtl(FAR struct ar_i2c_priv_s *priv, uint32_t tl)
{
    if (NULL != priv)
    {
        ar_i2c_putreg32(priv, I2C_IC_RX_TL, tl);
        return true;
    }
    else
    {
        return false;
    }
}

static uint8_t ar_i2c_set_txtl(struct ar_i2c_priv_s *priv, uint32_t tl)
{
    if (NULL != priv)
    {
        ar_i2c_putreg32(priv, I2C_IC_TX_TL, tl);
        return true;
    }
    else
    {
        i2cinfo("Can not get right ar_i2c_priv_s");
    }
    return false;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

static int ar_i2c_isr_process(struct ar_i2c_priv_s *priv)
{
    uint32_t status;

    uint32_t _abort;

    // status = ar_i2c_getreg32(priv, I2C_IC_INTR_MASK);
    // i2cinfo("I2C_IC_INTR_MASK = %04x \r\n", status);

    // 查询 I2C 中断状态寄存器的状态, 这个是存储
    status = ar_i2c_getreg32(priv, I2C_IC_INTR_STAT);

    _abort = ar_i2c_getreg32(priv, I2C_IC_TX_ABRT_SOURCE);

    ar_i2c_clearinterrupts(priv);

    uint8_t isout = 0;

    priv->status = status;
    // /* --------------------- Start of I2C protocol handling -------------------- */

    if ((status & IC_INTR_R_TX_ABRT) && _abort)
    {
        priv->abort_source = _abort;
        isout = 1;
    }

    else if (status & IC_INTR_R_RX_FULL)
    {

        switch (priv->msgc)
        {
        case 2:
        {
            uint8_t first_flag = priv->msgv[0].flags;
            uint8_t second_flag = priv->msgv[1].flags;

            if (!(first_flag & I2C_M_READ) && (second_flag & I2C_M_READ)) // write + read
            {
                int32_t count = 0;
                int32_t fifolength = ar_i2c_master_getrxfifolength(priv);

                // 读取所有的数据内容
                while (count < fifolength)
                {
                    uint8_t data = ar_i2c_getreg32(priv, I2C_IC_DATA_CMD) & 0xFF;

                    priv->i2c_int_data.rxbuf[priv->i2c_int_data.rx_alr_len++] = data;
                    count += 1;
                }

                if (priv->i2c_int_data.rx_alr_len < priv->i2c_int_data.rx_len)
                {
                    ar_i2c_fifo_read_data(priv);
                }
                else
                {

                    ar_i2c_modifyreg32(priv, I2C_IC_INTR_MASK, IC_INTR_R_RX_FULL, 0);
                    isout = 1;
                }
            }
            else
            {
                i2cerr("error, should not happen");
                isout = 1;
            }
            break;
        }
        default:
            i2cerr("Should not happen or not support \r\n");
            isout = 1;
            priv->msgc = 0;
            break;
        }
    }
    else if (status & (IC_INTR_R_TX_EMPTY))
    {

        /* The first event after the address byte is sent will be either TXIS
        * or NACKF so it's safe to set the astart flag to false on
        * the first TXIS event to indicate that it is no longer necessary to
        * check for address validity.
        */
        if (priv->i2c_int_data.tx_alr_len < priv->i2c_int_data.tx_len)
        {
            ar_i2c_load_fifo_write_data(priv);
        }
        else
        {
            ar_i2c_modifyreg32(priv, I2C_IC_INTR_MASK, IC_INTR_R_TX_EMPTY, 0);
        }
    }

    // 结束标志可能会在 TX_Empty 完成之后出现, 也可能与 RX_FULL 同时存在
    if (status & IC_INTR_R_STOP_DET) // STOP
    {
        i2cinfo("stop \r\n");
        isout = 1;
    }

    /* --------------------- End of I2C protocol handling -------------------- */

    /* Message Handling
     *
     * Transmission of the whole message chain has been completed. We have to
     * terminate the ISR and wake up ar_i2c_process() that is waiting for
     * the ISR cycle to handle the sending/receiving of the messages.
     */
    if (isout)
    {

        /* clear pointer to message content to reflect we are done
        * with the current transaction */

        priv->msgv = NULL;

        // priv->status = 0;
        // priv->abrt_source = 0;
        ar_i2c_clearinterrupts(priv);
        ar_i2c_disable_all_interrupts(priv);

        /* If a thread is waiting then inform it transfer is complete */

        if (priv->intstate == INTSTATE_WAITING)
        {
            sem_post(&priv->sem_isr);
            priv->intstate = INTSTATE_DONE;
        }
    }

    return OK;
}

static int ar_i2c_isr(int irq, void *context, FAR void *arg)
{
    struct ar_i2c_priv_s *priv = (struct ar_i2c_priv_s *)arg;

    DEBUGASSERT(priv != NULL);
    return ar_i2c_isr_process(priv);
}

static inline void ar_i2c_clearinterrupts(struct ar_i2c_priv_s *priv)
{
    ar_i2c_getreg32(priv, I2C_IC_CLR_INTR);
    ar_i2c_getreg32(priv, I2C_IC_CLR_TX_ABRT);
}

static inline void ar_i2c_clear_rxunder_interrupts(struct ar_i2c_priv_s *priv)
{
    ar_i2c_getreg32(priv, I2C_IC_CLR_RX_UNDER);
}

static inline void ar_i2c_clear_interrupts(struct ar_i2c_priv_s *priv, uint8_t offset)
{
    ar_i2c_getreg32(priv, offset);
}

static inline void ar_i2c_disable_all_interrupts(struct ar_i2c_priv_s *priv)
{
    ar_i2c_modifyreg32(priv, I2C_IC_INTR_MASK, IC_INTR_M_MASK, 0);
}

static inline void ar_i2c_enableinterrupts(struct ar_i2c_priv_s *priv)
{
    ar_i2c_modifyreg32(priv, I2C_IC_INTR_MASK, 0, IC_INTR_M_MASK);
}

static inline void ar_i2c_enable_rxfull_interrupts(struct ar_i2c_priv_s *priv)
{
    ar_i2c_modifyreg32(priv, I2C_IC_INTR_MASK, 0, IC_INTR_R_RX_FULL);
}

static inline void ar_i2c_disable_txempty_interrupts(struct ar_i2c_priv_s *priv)
{
    ar_i2c_modifyreg32(priv, I2C_IC_INTR_MASK, IC_INTR_M_TX_EMPTY, 0);
}

/**
 * 
*/
static inline void ar_i2c_sendstart(FAR struct ar_i2c_priv_s *priv)
{

    /* Set the private "current message" data used in protocol processing.
    *
    * ptr:   A pointer to the start of the current message buffer.  This is
    *        advanced after each byte in the current message is transferred.
    *
    * dcnt:  A running counter of the bytes in the current message waiting to be
    *        transferred.  This is decremented each time a byte is transferred.
    *        The hardware normally accepts a maximum of 255 bytes per transfer
    *        but can support more via the RELOAD mechanism.  If dcnt initially
    *        exceeds 255, the RELOAD mechanism will be enabled automatically.
    *
    * flags: Used to characterize handling of the current message.
    *
    *  The default flags value is 0 which specifies:
    *
    *   - A transfer direction of WRITE (R/W bit = 0)
    *   - RESTARTs between all messages
    *
    *  The following flags can be used to override this behavior as follows:
    *
    *   - I2C_M_READ: Sets the transfer direction to READ (R/W bit = 1)
    *   - I2C_M_NORESTART: Prevents a RESTART from being issued prior to the
    *      transfer of the message (where allowed by the protocol).
    *
    */

    // priv->flags = priv->msgv->flags;

    /* Set the (7 bit) address.
     * 10 bit addressing is not yet supported.
     */
    // ar_i2c_set_7bit_address(priv);

    ar_i2c_clearinterrupts(priv);
    ar_i2c_disable_all_interrupts(priv);
}

static inline void ar_i2c_sendstop(FAR struct ar_i2c_priv_s *priv)
{
    /* DEBUG... */
    ar_i2c_modifyreg32(priv, I2C_IC_DATA_CMD, 0, IC_DATA_CMD_STOP);
}

static void ar_i2c_msg_count_1_config_write(struct ar_i2c_priv_s *priv, FAR struct i2c_msg_s *msgs)
{
    priv->i2c_int_data.rxbuf = NULL;
    priv->i2c_int_data.rx_alr_len = 0;
    priv->i2c_int_data.rx_len = 0;

    priv->i2c_int_data.txbuf = msgs[0].buffer;
    priv->i2c_int_data.tx_alr_len = 0;
    priv->i2c_int_data.tx_len = msgs[0].length;

    // address & frequency are fixed
    priv->addr = msgs[0].addr;
    priv->frequency = msgs[0].frequency;

    priv->msgv = msgs;
    priv->msgc = 1;

    // 1. 设置地址
    /* Set the (7 bit) address.
     * 10 bit addressing is not yet supported.
     */
    ar_i2c_set_7bit_address(priv);

    /* 
     * 暂时不支持频率的变换， 也不支持 slave 地址的更改
     */
    ar_i2c_setclock(priv, priv->frequency);

    /**
     * 设置状态为等待状态
    */
    priv->intstate = INTSTATE_WAITING;

    ar_i2c_enable(priv);

    ar_i2c_load_fifo_write_data(priv);
}

static void ar_i2c_msg_count_2_config_write_read(struct ar_i2c_priv_s *priv, FAR struct i2c_msg_s *msgs)
{
    priv->i2c_int_data.txbuf = msgs[0].buffer;
    priv->i2c_int_data.tx_alr_len = 0;
    priv->i2c_int_data.tx_len = msgs[0].length;

    priv->i2c_int_data.rxbuf = msgs[1].buffer;
    priv->i2c_int_data.rx_alr_len = 0;
    priv->i2c_int_data.rx_len = msgs[1].length;

    // address & frequency are fixed
    priv->addr = msgs[0].addr;
    priv->frequency = msgs[0].frequency;

    priv->msgv = msgs;
    priv->msgc = 2;


    // 1. 设置地址
    /* Set the (7 bit) address.
     * 10 bit addressing is not yet supported.
     */
    ar_i2c_set_7bit_address(priv);

    /* 
     * 暂时不支持频率的变换， 也不支持 slave 地址的更改
     */
    ar_i2c_setclock(priv, priv->frequency);

    /**
     * 设置状态为等待状态
    */
    priv->intstate = INTSTATE_WAITING;

    ar_i2c_enable(priv);

    // 读操作之前先写入地址S
    unsigned int data = priv->i2c_int_data.txbuf[0];
    i2c_write_byte(priv, data);

    // 读操作
    ar_i2c_fifo_read_data(priv);
}

static int
ar_i2c_process(FAR struct i2c_master_s *dev, FAR struct i2c_msg_s *msgs, int count)
{
    struct ar_i2c_inst_s *inst = (struct ar_i2c_inst_s *)dev;

    FAR struct ar_i2c_priv_s *priv = inst->priv;

    int errval = 0;
    int waitrc = 0;

    ASSERT(count);

    priv->abort_source = 0;

    priv->status = 0;

    priv->msgc = 0;

    /* Wait for any STOP in progress */
    ar_i2c_sem_waitstop(priv);

    /* Clear any pending error interrupts */
    ar_i2c_disable(priv);

    ar_i2c_clearinterrupts(priv);

    ar_i2c_disable_all_interrupts(priv);

    switch (count)
    {
    case 1:
    {
        uint8_t flag = msgs[0].flags;

        if (!(flag & I2C_M_READ)) // Write
        {
            ar_i2c_msg_count_1_config_write(priv, msgs);
        }
        else
        {
            i2cinfo("count = 1, read should not happend ");
        }
        break;
    }
    case 2:
    {
        uint8_t first_flag = msgs[0].flags;
        uint8_t second_flag = msgs[1].flags;

        if (!(first_flag & I2C_M_READ) && (second_flag & I2C_M_READ)) // write + read
        {
            ar_i2c_msg_count_2_config_write_read(priv, msgs);
        }
        else
        {
            i2cerr("count = 2, rnot write + read pattern, not support");
        }
        break;
    }
    default:
        i2cerr("Not Support");
        break;
    }

    /* Wait for the ISR to tell us that the transfer is complete by attempting
    * to grab the semaphore that is initially locked by the ISR.  If the ISR
    * does not release the lock so we can obtain it here prior to the end of
    * the timeout period waitdone returns error and we report a timeout.
    */
    waitrc = ar_i2c_sem_waitdone(priv);

    /* Status after a normal / good exit is usually 0x00000001, meaning the TXE
    * bit is set.  That occurs as a result of the I2C_TXDR register being
    * empty, and it naturally will be after the last byte is transmitted.
    * This bit is cleared when we attempt communications again and re-enable
    * the peripheral.  The priv->status field can hold additional information
    * like a NACK, so we reset the status field to include that information.
    */
    if (waitrc < 0)
    {
        i2cerr("ETIMEDOUT \r\n");
        /* Connection timed out */
        errval = ETIMEDOUT;
    }

    else if ((priv->status & IC_INTR_R_TX_ABRT) && priv->abort_source)
    {
        // i2cerr("priv->abort_source =  %x　\r\n", priv->abort_source);
        if (priv->abort_source & IC_TX_ABRT_7B_ADDR_NOACK)
        {
            // "Bad address"
            errval = EFAULT;
        }
        else if (priv->abort_source & IC_TX_ABRT_TXDATA_NOACK)
        {
            // "Broken pipe"
            errval = EPIPE;
        }
        else
        {
            // "I/O error"
            errval = EIO;
        }

        for(uint8_t i = 0 ; i < 20; i++ )
        {
            up_udelay(10);
            uint32_t wait_stop_bit = ar_i2c_getreg32(priv,I2C_IC_RAW_INTR_STAT);
            if((wait_stop_bit & IC_INTR_R_STOP_DET))
            {
                break;
            }
        }

    }
    else
    {
        errval = 0;
    }

    //i2cinfo("out errval = %d \r\n", errval);
    ar_i2c_sem_post(dev);

    return -errval;
}

static int ar_i2c_transfer(FAR struct i2c_master_s *dev, FAR struct i2c_msg_s *msgs,
                           int count)
{
    ar_i2c_sem_wait(dev); /* ensure that address or flags don't change meanwhile */

    return ar_i2c_process(dev, msgs, count);
}

/************************************************************************************
 * Name: ar_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ************************************************************************************/

static int ar_i2c_init(FAR struct ar_i2c_priv_s *priv)
{

    Reg_Write32_Mask(SFR_PAD_CTRL7_REG, 0, BIT(14) | BIT(15) | BIT(16) | BIT(17));
    putreg32(SFR_PAD_CTRL7_REG, (0xF << 14));

    ar_i2c_disable(priv); /* 1: disable i2c */

    ar_i2c_modifyreg32(priv, I2C_IC_CON, (7 << 2), 0); //clear the speed with on 0

    ar_i2c_modifyreg32(priv, I2C_IC_CON,
                       IC_CON_IC_10BITADDR_MASTER | IC_CON_IC_10BITADDR_SLAVE | IC_CON_IC_RESTART_EN,
                       IC_CON_IC_SLAVE_DISABLE | IC_CON_SPEED_STANDARD_MODE | IC_CON_MASTER_MODE);

    ar_i2c_putreg32(priv, I2C_IC_FS_SPKLEN, 1);

    ar_i2c_putreg32(priv, I2C_IC_SS_SCL_HCNT, 650);
    ar_i2c_putreg32(priv, I2C_IC_SS_SCL_LCNT, 650);

    ar_i2c_putreg32(priv, I2C_IC_TAR, 0x00 & 0x7F); // set target

    ar_i2c_putreg32(priv, I2C_IC_RX_TL, 0); // juset default
    ar_i2c_putreg32(priv, I2C_IC_TX_TL, 0); // juset default

    ar_i2c_disable_all_interrupts(priv);
    ar_i2c_clearinterrupts(priv);

    irq_attach(priv->config->ev_irq, ar_i2c_isr, priv);
    up_enable_irq(priv->config->ev_irq);

    ar_i2c_enable(priv);

    return OK;
}

/************************************************************************************
 * Name: ar_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ************************************************************************************/

FAR struct i2c_master_s *ar_i2cbus_initialize(int port)
{
    struct ar_i2c_priv_s *priv = NULL; /* private data of device with multiple instances */
    struct ar_i2c_inst_s *inst = NULL; /* device, single instance */
    int irqs;
    /* Get I2C private structure */
    switch (port)
    {
    case 1:
        priv = (struct ar_i2c_priv_s *)&ar_i2c1_priv;
        break;

    case 2:
        priv = (struct ar_i2c_priv_s *)&ar_i2c2_priv;
        break;

    case 3:
        priv = (struct ar_i2c_priv_s *)&ar_i2c3_priv;
        break;

    case 4:
        priv = (struct ar_i2c_priv_s *)&ar_i2c4_priv;
        break;

    default:
        return NULL;
    }

    /* Allocate instance */

    if (!(inst = kmm_malloc(sizeof(struct ar_i2c_inst_s))))
    {
        return NULL;
    }
    /* Initialize instance */

    inst->ops = &ar_i2c_ops;
    inst->priv = priv;

    /* Init private data for the first time, increment refs count,
    * power-up hardware and configure GPIOs.
    */

    irqs = enter_critical_section();

    if ((volatile int)priv->refs++ == 0)
    {
        ar_i2c_sem_init((struct i2c_master_s *)inst);
        ar_i2c_init(priv);
    }

    leave_critical_section(irqs);

    return (struct i2c_master_s *)inst;
}

/************************************************************************************
 * Name: ar_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ************************************************************************************/

int ar_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
    int irqs;
    ASSERT(dev);

    /* Decrement refs and check for underflow */

    if (((struct ar_i2c_inst_s *)dev)->priv->refs == 0)
    {
        return ERROR;
    }

    irqs = enter_critical_section();

    if (--((struct ar_i2c_inst_s *)dev)->priv->refs)
    {
        leave_critical_section(irqs);
        kmm_free(dev);
        return OK;
    }

    leave_critical_section(irqs);

    /* Disable power and other HW resource (GPIO's) */

    ar_i2c_deinit(((struct ar_i2c_inst_s *)dev)->priv);

    /* Release unused resources */

    ar_i2c_sem_destroy((struct i2c_master_s *)dev);

    kmm_free(dev);
    return OK;
}
