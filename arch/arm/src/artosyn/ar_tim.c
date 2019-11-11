/************************************************************************************
 * arm/arm/src/artosyn/ar_tim.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With modifications and updates by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/timers/timer.h>
#include <nuttx/drivers/pwm.h>


#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "ar_tim.h"
#include "chip/ar_define.h"
#include "chip/ar_config.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int ar_tim_handler(int irq, void * context, void * arg);

/* "Lower half" driver methods **********************************************/

static int ar_tim_start(FAR struct timer_lowerhalf_s *lower);
static int ar_tim_stop(FAR struct timer_lowerhalf_s *lower);
static int ar_tim_settimeout(FAR struct timer_lowerhalf_s *lower,
                            uint32_t timeout);
static void ar_tim_setcallback(FAR struct timer_lowerhalf_s *lower,
                              tccb_t callback, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = ar_tim_start,
  .stop        = ar_tim_stop,
  .getstatus   = NULL,
  .settimeout  = ar_tim_settimeout,
  .setcallback = ar_tim_setcallback,
  .ioctl       = NULL,
};


struct ar_lowerhalf_s
{
    FAR const struct                timer_ops_s *ops;        /* Lower half operations */
    int32_t                         timernumber;                      /* 0~23 */
    tccb_t                          callback;   /* Current user interrupt callback */
    FAR void                        *arg;        /* Argument passed to upper half callback */
    bool                            started;    /* True: Timer has been started */
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t ar_tim_getreg32(uint32_t addr)
{
    return getreg32(addr);
}

static inline void ar_tim_putreg32(uint32_t addr, uint32_t value)
{
    putreg32(value, addr);
}

static inline void ar_tim_modifyreg32(uint32_t addr, uint32_t clearbits, uint32_t setbits)
{
    modifyreg32(addr, clearbits, setbits);
}

static inline uint32_t _ar_getregaddr_loadcount(ENUM_AR_TIMER_NUM timernumber)
{
    uint8_t u8_TimNum = timernumber/8;
    uint8_t u8_TimGroup = timernumber%8;

    uint32_t u32_TimBaseAddr = 0;
    uint32_t u32_Offset = 0;

    u32_TimBaseAddr = (u8_TimGroup==0)? BASE_ADDR_TIMER0:((u8_TimGroup==1)? BASE_ADDR_TIMER1:BASE_ADDR_TIMER2);
    u32_Offset = (CNT1_0+(u8_TimNum*0x14));

    return u32_TimBaseAddr + u32_Offset;
}

static inline uint32_t _ar_getregaddr_loadcount2(ENUM_AR_TIMER_NUM timernumber)
{
    uint8_t u8_TimNum = timernumber/8;
    uint8_t u8_TimGroup = timernumber%8;

    uint32_t u32_TimBaseAddr = 0;
    uint32_t u32_Offset = 0;

    u32_TimBaseAddr = (u8_TimGroup==0)? BASE_ADDR_TIMER0:((u8_TimGroup==1)? BASE_ADDR_TIMER1:BASE_ADDR_TIMER2);
    u32_Offset = (CNT2_0+(u8_TimNum*0x04));

    return u32_TimBaseAddr + u32_Offset;
}

static inline uint32_t _ar_getregaddr_control(ENUM_AR_TIMER_NUM timernumber)
{
    uint8_t u8_TimNum = timernumber/8;
    uint8_t u8_TimGroup = timernumber%8;

    uint32_t u32_TimBaseAddr = 0;
    uint32_t u32_Offset = 0;

    u32_TimBaseAddr = (u8_TimGroup==0)? BASE_ADDR_TIMER0:((u8_TimGroup==1)? BASE_ADDR_TIMER1:BASE_ADDR_TIMER2);
    u32_Offset = (CTRL_0+(u8_TimNum*0x14));

    return u32_TimBaseAddr + u32_Offset;
}

static inline uint32_t _ar_getregaddr_neoi(ENUM_AR_TIMER_NUM timernumber)
{
    uint8_t u8_TimNum = timernumber/8;
    uint8_t u8_TimGroup = timernumber%8;

    uint32_t u32_TimBaseAddr = 0;
    uint32_t u32_Offset = 0;

    u32_TimBaseAddr = (u8_TimGroup==0)? BASE_ADDR_TIMER0:((u8_TimGroup==1)? BASE_ADDR_TIMER1:BASE_ADDR_TIMER2);
    u32_Offset = (TMRNEOI_0+(u8_TimNum*0x14));

    return u32_TimBaseAddr + u32_Offset;
}

static void _ar_tim_enableint(ENUM_AR_TIMER_NUM timernumber)
{
    uint32_t addr = _ar_getregaddr_control(timernumber);

    ar_tim_modifyreg32(addr, 0, TIME_INT_MASK);
}

static void _ar_tim_disableint(ENUM_AR_TIMER_NUM timernumber)
{
    uint32_t addr = _ar_getregaddr_control(timernumber);

    ar_tim_modifyreg32(addr, TIME_INT_MASK, 0);
}

static void _ar_tim_enabletimer(ENUM_AR_TIMER_NUM timernumber, uint32_t ctlvalue)
{
    uint32_t addr = _ar_getregaddr_control(timernumber);

    ar_tim_modifyreg32(addr, 0, ctlvalue);
}

static void _ar_tim_disabletimer(ENUM_AR_TIMER_NUM timernumber, uint32_t ctlvalue)
{
    uint32_t addr = _ar_getregaddr_control(timernumber);

    ar_tim_modifyreg32(addr, ctlvalue, 0);
}


static int _ar_tim_setisr(ENUM_AR_TIMER_NUM timernumber,
             xcpt_t handler, void *arg)
{
    int vectorno;

    if (TIMER_NUM0 > timernumber || TIMER_NUM23 < timernumber)
        return -EINVAL;

    vectorno = AR_IRQ_TIM0 + (timernumber - TIMER_NUM0);

    /* Disable interrupt when callback is removed */

    if (!handler)
    {
        up_disable_irq(vectorno);
        irq_detach(vectorno);
        return OK;
    }

    /* Otherwise set callback and enable interrupt */

    irq_attach(vectorno, handler, arg);
    up_enable_irq(vectorno);

#ifdef CONFIG_ARCH_IRQPRIO
    /* Set the interrupt priority */

    up_prioritize_irq(vectorno, NVIC_SYSH_PRIORITY_DEFAULT);
#endif

    return OK;
}

 /**
 * @brief    clear timer interrupt
 * @note     none
 */
void _ar_tim_clearint(ENUM_AR_TIMER_NUM timernumber)
{
    uint32_t addr = _ar_getregaddr_neoi(timernumber);

    ar_tim_getreg32(addr);
}

void _ar_config_cnt(ENUM_AR_TIMER_NUM timernumber, uint32_t cnt1_value, uint32_t cnt2_value)
{
    uint32_t cnt1_addr = _ar_getregaddr_loadcount(timernumber);
    uint32_t cnt2_addr = _ar_getregaddr_loadcount2(timernumber);

    ar_tim_putreg32(cnt1_addr, cnt1_value);
    ar_tim_putreg32(cnt2_addr, cnt2_value);
}

static int _ar_settimeout(ENUM_AR_TIMER_NUM timernumber, uint32_t timeout)
{
    int freq = (CPU0_CPU1_CORE_PLL_CLK >> 1)/1000000;
    uint32_t count = freq *timeout;

    _ar_config_cnt(timernumber, count, 0);

    return OK;
}


/****************************************************************************
 * Name: ar_tim_handler
 *
 * Description:
 *   timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/

static int ar_tim_handler(int irq, void *context, void *arg)
{
    FAR struct ar_lowerhalf_s *priv = (struct ar_lowerhalf_s *) arg;
    uint32_t next_interval_us = 0;

    _ar_tim_clearint(priv->timernumber);

    if (priv->callback(&next_interval_us, priv->arg))
    {
        if (next_interval_us > 0)
        {
            _ar_settimeout(priv->timernumber, next_interval_us);
        }
    }
    else
    {
        ar_tim_stop((struct timer_lowerhalf_s *)priv);
    }

    return OK;
}

/****************************************************************************
 * Name: ar_tim_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ar_tim_start(FAR struct timer_lowerhalf_s *lower)
{
    FAR struct ar_lowerhalf_s *priv = (FAR struct ar_lowerhalf_s *)lower;
    // uint32_t addr;

    if (!priv->started)
    {
        // set mod
        _ar_tim_enabletimer(priv->timernumber, TIME_ENABLE | USER_DEFINED);

        if (priv->callback != NULL)
        {
            _ar_tim_setisr(priv->timernumber, ar_tim_handler, priv);
            _ar_tim_enableint(priv->timernumber);
        }

        priv->started = true;
        return OK;
    }

    /* Return EBUSY to indicate that the timer was already running */

    return -EBUSY;
}

/****************************************************************************
 * Name: ar_tim_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ar_tim_stop(struct timer_lowerhalf_s *lower)
{
    struct ar_lowerhalf_s *priv = (struct ar_lowerhalf_s *)lower;
    // uint32_t addr;

    if (priv->started)
    {
        _ar_tim_disabletimer(priv->timernumber, TIME_ENABLE | USER_DEFINED);
        _ar_tim_disableint(priv->timernumber);
        _ar_tim_setisr(priv->timernumber, NULL, NULL);

        priv->started = false;
        return OK;
    }

    /* Return ENODEV to indicate that the timer was not running */

    return -ENODEV;
}

/****************************************************************************
 * Name: ar_tim_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in microseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ar_tim_settimeout(FAR struct timer_lowerhalf_s *lower, uint32_t timeout)
{
    FAR struct ar_lowerhalf_s *priv = (FAR struct ar_lowerhalf_s *)lower;
    return _ar_settimeout(priv->timernumber, timeout);
}

static void ar_tim_setcallback(FAR struct timer_lowerhalf_s *lower,
                              tccb_t callback, FAR void *arg)
{
    FAR struct ar_lowerhalf_s *priv = (FAR struct ar_lowerhalf_s *)lower;

    irqstate_t flags = enter_critical_section();

    /* Save the new callback */

    priv->callback = callback;
    priv->arg      = arg;

    if (callback != NULL && priv->started)
    {
        _ar_tim_setisr(priv->timernumber, ar_tim_handler, priv);
        _ar_tim_enableint(priv->timernumber);
    }
    else
    {
        _ar_tim_disableint(priv->timernumber);
        _ar_tim_setisr(priv->timernumber, NULL, NULL);
    }

    leave_critical_section(flags);
}

static struct ar_lowerhalf_s g_tim1_lowerhalf =
{
    .ops            = &g_timer_ops,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ar_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *     form /dev/timer0
 *   timer - the timer's number.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int ar_timer_initialize(FAR const char *devpath, int timer)
{
    FAR struct ar_lowerhalf_s *lower;

    switch (timer)
    {
    case TIMER_NUM1:
        lower = &g_tim1_lowerhalf;
        break;

    default:
        return -ENODEV;
    }

    /* Initialize the elements of lower half state structure */

    lower->started          = false;
    lower->callback         = NULL;
    lower->timernumber      = timer;

    /* Register the timer driver as /dev/timerX.  The returned value from
    * timer_register is a handle that could be used with timer_unregister().
    * REVISIT: The returned handle is discard here.
    */

    FAR void *drvr = timer_register(devpath,
            (FAR struct timer_lowerhalf_s *)lower);
    if (drvr == NULL)
    {
        /* The actual cause of the failure may have been a failure to allocate
        * perhaps a failure to register the timer driver (such as if the
        * 'depath' were not unique).  We know here but we return EEXIST to
        * indicate the failure (implying the non-unique devpath).
        */

        return -EEXIST;
    }

    return OK;
}

#if 0


#endif

static int ar_pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int ar_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int ar_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info);
static int ar_pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int ar_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/* This structure represents the state of one PWM timer */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = ar_pwm_setup,
  .shutdown    = ar_pwm_shutdown,
  .start       = ar_pwm_start,
  .stop        = ar_pwm_stop,
  .ioctl       = ar_pwm_ioctl,
};

struct ar_pwmtimer_s
{
    FAR const struct    pwm_ops_s *ops;     /* PWM operations */
    int32_t             timernumber;        /* 0~23 */
    uint32_t            frequency;          /* Current frequency setting */
    uint32_t            irq;
};

static struct ar_pwmtimer_s g_pwm1dev =
{
    .ops                  = &g_pwmops,
    .timernumber          = 1,
    .frequency            = 0,
};

/* Static Function Prototypes
 ****************************************************************************/
/* Register access */

/****************************************************************************
 * Name: ar_pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   APB1 or 2 clocking for the GPIOs has already been configured by the RCC
 *   logic at power up.
 *
 ****************************************************************************/

static int ar_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
//    FAR struct ar_pwmtimer_s *priv = (FAR struct ar_pwmtimer_s *)dev;

    return 0;
    /* TODO:config gpio */
}

/****************************************************************************
 * Name: ar_pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int ar_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
//   FAR struct ar_pwmtimer_s *priv = (FAR struct ar_pwmtimer_s *)dev;
//   uint32_t pincfg;
//  int i;

  pwminfo("TIM%u\n", priv->timernumber);

  /* Make sure that the output has been stopped */
  ar_pwm_stop(dev);

  /* TODO: Then put the GPIO pins back to the default state */


  return OK;
}

/* Timer management */
static int ar_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info)
{
    int ret = OK;
    FAR struct ar_pwmtimer_s *priv = (FAR struct ar_pwmtimer_s *)dev;

    int freq = (CPU0_CPU1_CORE_PLL_CLK >> 1)/1000000;

    uint32_t count1;
    uint32_t count2;
    uint32_t countsum;

    if (0 >= info->frequency || info->duty == 0 || info->duty >=65536)
        return -EPERM;

    countsum = freq/info->frequency;
    count2 = (countsum/65536)*info->duty;
    count1 = countsum - count2;

    /* config count and count1 reg */
    _ar_config_cnt(priv->timernumber, count1, count2);

    /* config control reg */
    _ar_tim_enabletimer(priv->timernumber, TIME_ENABLE | USER_DEFINED | TIME_PWM_ENABLE);

    /* disable irq */
    _ar_tim_disableint(priv->timernumber);
    _ar_tim_setisr(priv->timernumber, NULL, NULL);

    priv->frequency = info->frequency;

    return ret;
}

static int ar_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
    FAR struct ar_pwmtimer_s *priv = (FAR struct ar_pwmtimer_s *)dev;
    // uint32_t resetbit;
    // uint32_t regaddr;
    // uint32_t regval;
    irqstate_t flags;

    pwminfo("TIM%u\n", priv->timernumber);

    /* Disable interrupts momentary to stop any ongoing timer processing and
    * to prevent any concurrent access to the reset register.
    */

    flags = enter_critical_section();

    /* Stopped so frequency is zero */

    priv->frequency = 0;

    /* Disable further interrupts and stop the timer */
    _ar_tim_disabletimer(priv->timernumber, TIME_ENABLE | USER_DEFINED | TIME_PWM_ENABLE);
    _ar_tim_disableint(priv->timernumber);
    _ar_tim_setisr(priv->timernumber, NULL, NULL);

    /* Reset the timer - stopping the output and putting the timer back
    * into a state where pwm_start() can be called.
    */

    leave_critical_section(flags);
    return OK;
}

/****************************************************************************
* Name: ar_pwm_ioctl
*
* Description:
*   Lower-half logic may support platform-specific ioctl commands
*
* Input parameters:
*   dev - A reference to the lower half PWM driver state structure
*   cmd - The ioctl command
*   arg - The argument accompanying the ioctl command
*
* Returned Value:
*   Zero on success; a negated errno value on failure
*
****************************************************************************/

static int ar_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
    FAR struct ar_pwmtimer_s *priv = (FAR struct ar_pwmtimer_s *)dev;

    /* There are no platform-specific ioctl commands */

    pwminfo("TIM%u\n", priv->timernumber);
#endif
    return -ENOTTY;
}

static int ar_pwm_timinterrupt(int irq, void *context, FAR void *arg)
{
    return 0;
}


/****************************************************************************
* Public Functions
****************************************************************************/

/****************************************************************************
* Name: ar_pwminitialize
*
* Description:
*   Initialize one timer for use with the upper_level PWM driver.
*
* Input Parameters:
*   timer - A number identifying the timer use.
*
* Returned Value:
*   On success, a pointer to the ar lower half PWM driver is returned.
*   NULL is returned on any failure.
*
****************************************************************************/

FAR struct pwm_lowerhalf_s *ar_pwminitialize(int timer)
{
    FAR struct ar_pwmtimer_s *lower;

    pwminfo("TIM%u\n", timer);

    switch (timer)
    {

    case 1:
        lower = &g_pwm1dev;

        /* Attach but disable the TIM update interrupt */
        irq_attach(lower->irq, ar_pwm_timinterrupt, NULL);  // just for build
        up_disable_irq(lower->irq);
        break;

        /* Attach but disable the TIM8 update interrupt */
    default:
        pwmerr("ERROR: No such timer configured\n");
        return NULL;
    }

    return (FAR struct pwm_lowerhalf_s *)lower;
}

