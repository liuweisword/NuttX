#ifndef I2C_H
#define I2C_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    I2C_Component_0 = 0,
    I2C_Component_1,
    I2C_Component_2,
    I2C_Component_3,
    I2C_Component_4,
    I2C_Component_5
} EN_I2C_COMPONENT;

typedef enum
{
    I2C_Master_Mode = 0,
    I2C_Slave_Mode,
    I2C_Unknown_Mode,
} ENUM_I2C_Mode;

typedef enum
{
    I2C_Standard_Speed = 1,
    I2C_Fast_Speed,
    I2C_High_Speed,
    I2C_Unknown_Speed,
} ENUM_I2C_Speed;

typedef struct
{
    uint32_t txLen;         //
    uint32_t txAlrLen;      //
    uint8_t *txBuf;         //
    uint32_t txLenLast;     //
    uint32_t rxLen;         //
    uint32_t rxAlrLen;      //
    uint8_t *rxBuf;         //
    uint32_t rxAlrLanNum;
} STRU_I2C_INT_DATA;


#define SFR_PAD_CTRL7_REG (0x40B00098)


uint8_t I2C_Init(EN_I2C_COMPONENT en_component, ENUM_I2C_Mode en_i2cMode, uint16_t u16_i2cAddr, ENUM_I2C_Speed en_i2cSpeed);
uint8_t I2C_Master_WriteData(EN_I2C_COMPONENT en_component, uint16_t u16_i2cAddr, uint8_t* ptr_data, uint32_t u32_dataSize);
uint8_t I2C_Master_ReadData(EN_I2C_COMPONENT en_component, uint16_t u16_i2cAddr, uint8_t* ptr_subAddr, uint8_t u8_subAddrSize, uint8_t* ptr_data, uint32_t u32_dataSize);
uint8_t I2C_Master_ReadDataMfi(EN_I2C_COMPONENT en_component, uint16_t u16_i2cAddr, uint8_t* ptr_subAddr, uint8_t u8_subAddrSize, uint8_t* ptr_data, uint32_t u32_dataSize);
void I2C_Master_IntrSrvc(uint32_t u32_vectorNum);
int I2C_Master_GetBusyStatus(EN_I2C_COMPONENT en_component);
int I2C_Master_GetTxAbrtSource(EN_I2C_COMPONENT en_component);
int32_t I2C_Master_WaitTillIdle(EN_I2C_COMPONENT en_component, uint32_t timeOut);
int8_t I2C_Master_ClrTxAbrt(EN_I2C_COMPONENT en_component);


FAR struct i2c_master_s *ar_i2cbus_initialize(int port);

int ar_i2cbus_uninitialize(FAR struct i2c_master_s * dev);


#define MAX_I2C_CONTOLLER_NUMBER 6

#define BASE_ADDR_I2C0 (0x40200000)
#define BASE_ADDR_I2C1 (0x40240000)
#define BASE_ADDR_I2C2 (0x40280000)
#define BASE_ADDR_I2C3 (0x402c0000)
#define BASE_ADDR_I2C4 (0x40900000)
#define BASE_ADDR_I2C5 (0xA0080000)

// I2C Interrupt Mask Register
#define IC_INTR_M_MASK             (0xFFF)
#define IC_INTR_M_GEN_CALL         (1 << 11)
#define IC_INTR_M_START_DET        (1 << 10)
#define IC_INTR_M_STOP_DET         (1 << 9)
#define IC_INTR_M_ACTIVITY         (1 << 8)
#define IC_INTR_M_RX_DONE          (1 << 7)
#define IC_INTR_M_TX_ABRT          (1 << 6)
#define IC_INTR_M_RD_REQ           (1 << 5)
#define IC_INTR_M_TX_EMPTY         (1 << 4)
#define IC_INTR_M_TX_OVER          (1 << 3)
#define IC_INTR_M_RX_FULL          (1 << 2)
#define IC_INTR_M_RX_OVER          (1 << 1)
#define IC_INTR_M_RX_UNDER         (1 << 0)

// I2C Interrupt Status Register
#define IC_INTR_R_MASK             (0xFFF)
#define IC_INTR_R_GEN_CALL         (1 << 11)
#define IC_INTR_R_START_DET        (1 << 10)
#define IC_INTR_R_STOP_DET         (1 << 9)
#define IC_INTR_R_ACTIVITY         (1 << 8)
#define IC_INTR_R_RX_DONE          (1 << 7)
#define IC_INTR_R_TX_ABRT          (1 << 6)
#define IC_INTR_R_RD_REQ           (1 << 5)
#define IC_INTR_R_TX_EMPTY         (1 << 4)
#define IC_INTR_R_TX_OVER          (1 << 3)
#define IC_INTR_R_RX_FULL          (1 << 2)
#define IC_INTR_R_RX_OVER          (1 << 1)
#define IC_INTR_R_RX_UNDER         (1 << 0)


// I2C Status Register
#define IC_STATUS_MASK              (0x7F)
#define IC_STATUS_SLV_ACTIVITY      (1 << 6) 
#define IC_STATUS_MST_ACTIVITY      (1 << 5)
#define IC_STATUS_RFF               (1 << 4)
#define IC_STATUS_RFNE              (1 << 3)
#define IC_STATUS_TFE               (1 << 2)
#define IC_STATUS_TFNF              (1 << 1)
#define IC_STATUS_ACTIVITY          (1 << 0)

#define IC_STATUS_MST_ACTIVITY_IDLE     (0 << 5)
#define IC_STATUS_MST_ACTIVITY_NOTIDLE  (1 << 5)

// I2C ENABLE Register
#define IC_ENABLE_MASK              (0x3)
#define IC_ENABLE_ABORT             (1 << 1)
#define IC_ENABLE_ENABLE            (1 << 0)

// I2C Control Register
#define IC_CON_MASK                 (0x7F)
#define IC_CON_IC_SLAVE_DISABLE     (1 << 6)
#define IC_CON_IC_RESTART_EN        (1 << 5)
#define IC_CON_IC_10BITADDR_MASTER  (1 << 4)
#define IC_CON_IC_10BITADDR_SLAVE   (1 << 3)
#define IC_CON_SPEED_MASK           (3 << 1)
#define IC_CON_SPEED_STANDARD_MODE  (1 << 1)
#define IC_CON_SPEED_FAST_MODE      (2 << 1)
#define IC_CON_SPEED_HIGH_MODE      (3 << 1)
#define IC_CON_MASTER_MODE          (1 << 0)

// I2C tart register
#define IC_TAR_IC_10BITADDR_MASTER  (1 << 12)



// I2C Rx/Tx Data Buffer and Command Register
#define IC_DATA_CMD_MASK            (0x7FF)
#define IC_DATA_CMD_RESTART         (1 << 10)
#define IC_DATA_CMD_STOP            (1 << 9)
#define IC_DATA_CMD_CMD             (1 << 8)
#define IC_DATA_CMD_DAT_MASK        (0xFF)



// I2C Transmit Abort Source Register
#define IC_TX_ABRT_7B_ADDR_NOACK    (1 << 0)
#define IC_TX_ABRT_10ADDR1_NOACK    (1 << 1)
#define IC_TX_ABRT_10ADDR2_NOACK    (1 << 2)
#define IC_TX_ABRT_TXDATA_NOACK     (1 << 3)



#define I2C_FIFO_DEPTH           (8)

#define I2C_TX_FIFO_BUFFER_DEPTH (8)
#define I2C_RX_FIFO_BUFFER_DEPTH (8)

#define IC_RX_TL_DEF_VALUE       (4)
#define IC_TX_TL_DEF_VALUE       (4)



#define I2C_CMD_ID_START (0x10000)

#define ROUNDUP_DIVISION(a, b) ((a % b) ? ((a / b) + 1) : (a / b))

typedef enum
{
    I2C_CMD_SET_MODE = I2C_CMD_ID_START,
    I2C_CMD_SET_M_SPEED,
    I2C_CMD_SET_M_TARGET_ADDRESS,
    I2C_CMD_SET_S_SLAVE_ADDRESS,
    I2C_CMD_SET_M_WRITE_DATA,
    I2C_CMD_SET_M_READ_LAUNCH,
    I2C_CMD_SET_RX_TL,
    I2C_CMD_SET_INTR_ENABLE,
    I2C_CMD_SET_INTR_DISENABLE,
    I2C_CMD_GET_M_TX_FIFO_LENGTH,
    I2C_CMD_GET_M_RX_FIFO_LENGTH,
    I2C_CMD_GET_M_RX_FIFO_DATA,
    I2C_CMD_GET_M_IDLE,
    I2C_CMD_GET_INTR_STAT,
	I2C_CMD_GET_IC_CLR_TX_ABRT,
    I2C_CMD_GET_IC_TX_ABRT_SOURCE,
} ENUM_I2C_CMD_ID;

// Using macro define blow
// typedef struct
// {
//     unsigned int    IC_CON      ;       // 0x00
//     unsigned int    IC_TAR      ;       // 0x04
//     unsigned int    IC_SAR      ;       // 0x08
//     unsigned int    IC_HS_MADDR ;       // 0x0c
//     unsigned int    IC_DATA_CMD ;       // 0x10
//     unsigned int    IC_SS_SCL_HCNT;     // 0x14
//     unsigned int    IC_SS_SCL_LCNT;     // 0x18
//     unsigned int    IC_FS_SCL_HCNT;     // 0x1c
//     unsigned int    IC_FS_SCL_LCNT;     // 0x20
//     unsigned int    IC_HS_SCL_HCNT;     // 0x24
//     unsigned int    IC_HS_SCL_LCNT;     // 0x28
//     unsigned int    IC_INTR_STAT;       // 0x2c
//     unsigned int    IC_INTR_MASK;       // 0x30
//     unsigned int    IC_RAW_INTR_STAT;   // 0x34
//     unsigned int    IC_RX_TL    ;       // 0x38
//     unsigned int    IC_TX_TL    ;       // 0x3c
//     unsigned int    IC_CLR_INTR ;       // 0x40
//     unsigned int    IC_CLR_RX_UNDER;    // 0x44
//     unsigned int    IC_CLR_RX_OVER;     // 0x48
//     unsigned int    IC_CLR_TX_OVER;     // 0x4c
//     unsigned int    IC_CLR_RD_REQ;      // 0x50
//     unsigned int    IC_CLR_TX_ABRT;     // 0x54
//     unsigned int    IC_CLR_RX_DONE;     // 0x58
//     unsigned int    IC_CLR_ACTIVITY;    // 0x5c
//     unsigned int    IC_CLR_STOP_DET;    // 0x60
//     unsigned int    IC_CLR_START_DET;   // 0x64
//     unsigned int    IC_CLR_GEN_GALL;    // 0x68
//     unsigned int    IC_ENABLE   ;       // 0x6c
//     unsigned int    IC_STATUS   ;       // 0x70
//     unsigned int    IC_TXFLR    ;       // 0x74
//     unsigned int    IC_RXFLR    ;       // 0x78
//     unsigned int    IC_SDA_HOLD ;       // 0x7c
//     unsigned int    IC_TX_ABRT_SOURCE;  // 0x80
//     unsigned int    IC_SLV_DATA_NACK_ONLY;// 0x84
//     unsigned int    IC_DMA_CR   ;       // 0x88
//     unsigned int    IC_DMA_TDLR ;       // 0x8c
//     unsigned int    IC_DMA_RDLR ;       // 0x90
//     unsigned int    IC_SDA_SETUP;       // 0x94
//     unsigned int    IC_ACK_GENERAL_CALL;// 0x98
//     unsigned int    IC_ENABLE_STATUS;   // 0x9c
//     unsigned int    IC_FS_SPKLEN;       // 0xa0
//     unsigned int    IC_HS_SPKLEN;       // 0xa4
//     unsigned int    reserved[19];       // 0xa8 ~ 0xf0
//     unsigned int    IC_COMP_PARAM_1;    // 0xf4
//     unsigned int    IC_COMP_VERSION;    // 0xf8
//     unsigned int    IC_COMP_TYPE;       // 0xfc
// } STRU_I2C_Type;

#define   I2C_IC_CON                (0x00)
#define   I2C_IC_TAR                (0x04)
#define   I2C_IC_SAR                (0x08)
#define   I2C_IC_HS_MADDR           (0x0c)
#define   I2C_IC_DATA_CMD           (0x10)
#define   I2C_IC_SS_SCL_HCNT        (0x14)
#define   I2C_IC_SS_SCL_LCNT        (0x18)
#define   I2C_IC_FS_SCL_HCNT        (0x1c)
#define   I2C_IC_FS_SCL_LCNT        (0x20)
#define   I2C_IC_HS_SCL_HCNT        (0x24)
#define   I2C_IC_HS_SCL_LCNT        (0x28)
#define   I2C_IC_INTR_STAT          (0x2c)
#define   I2C_IC_INTR_MASK          (0x30)
#define   I2C_IC_RAW_INTR_STAT      (0x34)
#define   I2C_IC_RX_TL              (0x38)
#define   I2C_IC_TX_TL              (0x3c)
#define   I2C_IC_CLR_INTR           (0x40)
#define   I2C_IC_CLR_RX_UNDER       (0x44)
#define   I2C_IC_CLR_RX_OVER        (0x48)
#define   I2C_IC_CLR_TX_OVER        (0x4c)
#define   I2C_IC_CLR_RD_REQ         (0x50)
#define   I2C_IC_CLR_TX_ABRT        (0x54)
#define   I2C_IC_CLR_RX_DONE        (0x58)
#define   I2C_IC_CLR_ACTIVITY       (0x5c)
#define   I2C_IC_CLR_STOP_DET       (0x60)
#define   I2C_IC_CLR_START_DET      (0x64)
#define   I2C_IC_CLR_GEN_GALL       (0x68)
#define   I2C_IC_ENABLE             (0x6c)
#define   I2C_IC_STATUS             (0x70)
#define   I2C_IC_TXFLR              (0x74)
#define   I2C_IC_RXFLR              (0x78)
#define   I2C_IC_SDA_HOLD           (0x7c)
#define   I2C_IC_TX_ABRT_SOURCE     (0x80)
#define   I2C_IC_SLV_DATA_NACK_ONLY (0x84)
#define   I2C_IC_DMA_CR             (0x88)
#define   I2C_IC_DMA_TDLR           (0x8c)
#define   I2C_IC_DMA_RDLR           (0x90)
#define   I2C_IC_SDA_SETUP          (0x94)
#define   I2C_IC_ACK_GENERAL_CALL   (0x98)
#define   I2C_IC_ENABLE_STATUS      (0x9c)
#define   I2C_IC_FS_SPKLEN          (0xa0)
#define   I2C_IC_HS_SPKLEN          (0xa4)
#define   I2C_RESERVED              (0xa8)
#define   I2C_IC_COMP_PARAM_1       (0xf4)
#define   I2C_IC_COMP_VERSION       (0xf8)
#define   I2C_IC_COMP_TYPE          (0xfc)








typedef struct
{
    const uint32_t u32_i2cRegBaseAddr;
    ENUM_I2C_Mode  en_i2cMode;
    union
    {
        struct
        {
            uint16_t addr;
            ENUM_I2C_Speed  speed;
        } master;
        struct
        {
            uint16_t addr;
        } slave;
    } parameter;
} STRU_I2C_Controller;

void I2C_LL_Delay(unsigned int delay);
STRU_I2C_Controller* I2C_LL_GetI2CController(EN_I2C_COMPONENT en_i2cComponent);
uint8_t I2C_LL_IOCtl(STRU_I2C_Controller* ptr_i2cController, ENUM_I2C_CMD_ID en_CommandID, uint32_t* ptr_CommandVal);

typedef enum
{
    HAL_I2C_COMPONENT_0 = 0,
    HAL_I2C_COMPONENT_1,
    HAL_I2C_COMPONENT_2,
    HAL_I2C_COMPONENT_3,
    HAL_I2C_COMPONENT_4,
    HAL_I2C_COMPONENT_5,
    HAL_I2C_COMPONENT_MAX
} ENUM_HAL_I2C_COMPONENT;


typedef enum
{
    HAL_I2C_STANDARD_SPEED = 0,
    HAL_I2C_FAST_SPEED,
    HAL_I2C_HIGH_SPEED
} ENUM_HAL_I2C_SPEED;

// /**
// * @brief  The I2C initialization function which must be called before using the I2C controller.
// * @param  e_i2cComponent    The I2C controller number, the right number should be 0-4 and totally
// *                           5 I2C controllers can be used by application.
// *         u16_i2cAddr       16 bit I2C address of the target device.
// *         e_i2cSpeed        The I2C speed of the I2C clock of the I2C controller, the right value
// *                           should be standard (<100Kb/s), fast (<400Kb/s) and high(<3.4Mb/s).
// * @retval HAL_OK            means the initializtion is well done.
// *         HAL_I2C_ERR_INIT  means some error happens in the initializtion.
// * @note   Master mode only
// *         High speed mode has some system dependency and is especially affected by the circuit capacity
// *         and the I2C slave device ability.
// */

// HAL_RET_T HAL_I2C_MasterInit(ENUM_HAL_I2C_COMPONENT e_i2cComponent, 
//                              uint16_t u16_i2cAddr,
//                              ENUM_HAL_I2C_SPEED e_i2cSpeed);

// /**
// * @brief  The I2C data write function which can be used to send out I2C data by the I2C controller.
// * @param  e_i2cComponent          The I2C controller number, the right number should be 0-4 and totally
// *                                 5 I2C controllers can be used by application.
// *         u16_i2cAddr             16 bit I2C address of the target device.
// *         pu8_wrData              The transmit buffer pointer to be sent out by I2C bus. In normal case, the first 1 
// *                                 or 2 bytes should be the sub address to be accessed.
// *         u16_wrSize              The transmit buffer size in byte. 
// * @retval HAL_OK                  means the I2C data write is well done.
// *         HAL_I2C_ERR_WRITE_DATA  means some error happens in the I2C data write.
// * @note   u32_wrSize should be less than 6, this is the I2C fifo limit. There is some fifo full risk when u32_wrSize 
//           is larger than 6.
//           High speed mode has some system dependency and is especially affected by the circuit capacity.
// */

// HAL_RET_T HAL_I2C_MasterWriteData(ENUM_HAL_I2C_COMPONENT e_i2cComponent, 
//                                   uint16_t u16_i2cAddr,
//                                   const uint8_t *pu8_wrData,
//                                   uint32_t u32_wrSize,
//                                   uint32_t u32_timeOut);

// /**
// * @brief  The I2C data read function which can be used to read I2C data by the I2C controller.
// * @param  e_i2cComponent          The I2C controller number, the right number should be 0-4 and totally
// *                                 5 I2C controllers can be used by application.
// *         u16_i2cAddr             16 bit I2C address of the target device.
// *         pu8_wrData              The transmit buffer pointer to be sent out by I2C bus before read starts.
//                                   In normal case, the first 1 or 2 bytes should be the sub address to be accessed.
// *         u16_wrSize              The transmit buffer size in byte. 
//           pu8_rdData              The receive buffer pointer to hold the data in read operation.
//           u16_rdSize              The receive buffer size in byte.
// * @retval HAL_OK                  means the initializtion is well done.
// *         HAL_I2C_ERR_READ_DATA   means some error happens in the data read.
// * @note   High speed mode has some system dependency and is especially affected by the circuit capacity.
// */

// HAL_RET_T HAL_I2C_MasterReadData(ENUM_HAL_I2C_COMPONENT e_i2cComponent, 
//                                  uint16_t u16_i2cAddr,
//                                  const uint8_t *pu8_wrData,
//                                  uint8_t  u8_wrSize,
//                                  uint8_t *pu8_rdData,
//                                  uint32_t u32_rdSize,
//                                  uint32_t u32_timeOut);



#endif


