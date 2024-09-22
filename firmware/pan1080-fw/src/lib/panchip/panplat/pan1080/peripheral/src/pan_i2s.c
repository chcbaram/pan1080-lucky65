#include "PanSeries.h"
#include "pan_i2s.h"



/**
  * @brief  set I2s word select lenth,default 16 clk cycle
  * @param  i2s The base address of i2s module
  * @param  u8WordSel: sclk cycle select
                        I2S_WORD_SEL_16_CYCLE
                        I2S_WORD_SEL_24_CYCLE
                        I2S_WORD_SEL_32_CYCLE
  * @retval None
  */
void I2S_SetWordSelectLen(I2S_T *i2s,uint8_t u8WordSel)
{
    uint32_t u32tmpReg;
    u32tmpReg = i2s->CCR;
    u32tmpReg &= ~I2S_WORD_SELECT_LEN_Msk;
    u32tmpReg |= u8WordSel << I2S_WORD_SELECT_LEN_Pos;
    
    i2s->CCR = u32tmpReg;
}
/**
  * @brief  set I2s clk gating cycle,default:no gating
  * @param  i2s The base address of i2s module
  * @param  u8SclkSel: gating select:
                        I2S_CLK_GATING_SEL_0_CYCLE
                        I2S_CLK_GATING_SEL_12_CYCLE
                        I2S_CLK_GATING_SEL_16_CYCLE
                        I2S_CLK_GATING_SEL_24_CYCLE
                        I2S_CLK_GATING_SEL_32_CYCLE
  * @retval None
  */
void I2S_SetSclkGate(I2S_T *i2s,uint8_t u8SclkSel)
{
    uint32_t u32tmpReg;
    u32tmpReg = i2s->CCR;
    u32tmpReg &= ~I2S_SCLK_GATING_Msk;
    u32tmpReg |= u8SclkSel << I2S_SCLK_GATING_Pos;

    i2s->CCR = u32tmpReg;
}


/**
  * @brief  set I2s receive word length resolution,default:no word length
  * @param  i2s The base address of i2s module
  * @param  ch: i2s channel select
  * @param  u8WordLen: word length resolution select:
                                            I2S_WORD_LEN_IGNORE
                                            I2S_WORD_LEN_12_BIT_RESOLUTION
                                            I2S_WORD_LEN_16_BIT_RESOLUTION
                                            I2S_WORD_LEN_20_BIT_RESOLUTION
                                            I2S_WORD_LEN_24_BIT_RESOLUTION
                                            I2S_WORD_LEN_32_BIT_RESOLUTION
  * @retval None
  */
void I2S_SetRecWordLen(I2S_T *i2s,uint8_t ch,uint8_t u8WordLen)
{
    uint32_t u32tmpReg;
    u32tmpReg = i2s->CH[ch].RCR;
    u32tmpReg &= ~I2S_WORD_LENGTH_SET_Msk;
    u32tmpReg |= u8WordLen << I2S_WORD_LENGTH_SET_Pos;

    i2s->CH[ch].RCR = u32tmpReg;
}
/**
  * @brief  set I2s transmitter word length resolution,default:no word length
  * @param  i2s The base address of i2s module
  * @param  ch: i2s channel select
  * @param  u8WordLen: word length resolution select:
                                            I2S_WORD_LEN_IGNORE
                                            I2S_WORD_LEN_12_BIT_RESOLUTION
                                            I2S_WORD_LEN_16_BIT_RESOLUTION
                                            I2S_WORD_LEN_20_BIT_RESOLUTION
                                            I2S_WORD_LEN_24_BIT_RESOLUTION
                                            I2S_WORD_LEN_32_BIT_RESOLUTION
  * @retval None
  */
void I2S_SetTrmWordLen(I2S_T *i2s,uint8_t ch,uint8_t u8WordLen)
{
    uint32_t u32tmpReg;
    u32tmpReg = i2s->CH[ch].TCR;
    u32tmpReg &= ~I2S_WORD_LENGTH_SET_Msk;
    u32tmpReg |= u8WordLen << I2S_WORD_LENGTH_SET_Pos;

    i2s->CH[ch].TCR = u32tmpReg;
}

/**
  * @brief  Clear rx fifo over interrupt status
  * @param  i2s The base address of i2s module
  * @param  ch: i2s channel select
  * @retval NONE
  */
uint32_t I2S_ClearRxOverInt(I2S_T *i2s,uint8_t ch)    
{
    uint32_t u32tmpReg;
    u32tmpReg = i2s->CH[ch].ROR;
    return u32tmpReg;
}

/**
  * @brief  Clear tx fifo over interrupt status
  * @param  i2s The base address of i2s module
  * @param  ch: i2s channel select
  * @retval NONE
  */
uint32_t I2S_ClearTxOverInt(I2S_T *i2s,uint8_t ch)    
{
    uint32_t u32tmpReg;
    u32tmpReg = i2s->CH[ch].TOR;
    return u32tmpReg;
}
/**
  * @brief  I2S send data
  * @param  i2s The base address of i2s module
  * @param  ch: i2s channel select
  * @param  LeftBuf: left channel data buffer
  * @param  RightBuf: right channel data buffer
  * @param  Size: send data number
  * @retval NONE
  */
void I2S_Send(I2S_T *I2Sx,uint8_t Ch,uint32_t *LeftBuf,uint32_t *RightBuf,uint32_t Size)
{
    while(Size--){
        I2Sx->CH[Ch].LTHR_LRBR = (uint16_t)*LeftBuf;
        LeftBuf++;
        I2Sx->CH[Ch].RTHR_RRBR = (uint16_t)*RightBuf;
        RightBuf++;
    }
}

/**
  * @brief  I2S send data
  * @param  i2s The base address of i2s module
  * @param  ch: i2s channel select
  * @param  LeftBuf: left channel data buffer
  * @param  RightBuf: right channel data buffer
  * @param  Size: send data number
  * @retval NONE
  */
void I2S_Receive(I2S_T *I2Sx,uint8_t Ch,uint32_t *LeftBuf,uint32_t *RightBuf,uint32_t Size)
{
    while(Size--){
        *LeftBuf = (uint16_t)I2Sx->CH[Ch].LTHR_LRBR;;
        LeftBuf++;
        *RightBuf = (uint16_t)I2Sx->CH[Ch].RTHR_RRBR;;
        RightBuf++;
    }
}
