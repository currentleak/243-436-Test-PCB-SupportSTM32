/* drv8234.h - minimal I2C helpers for DRV8234 control */
#ifndef __DRV8234_H
#define __DRV8234_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint16_t address; /* 7-bit address (0x00-0x7F) */
} drv8234_t;

/* Generic I2C register access */
HAL_StatusTypeDef drv8234_write_reg(drv8234_t *dev, uint8_t reg, uint8_t value);
HAL_StatusTypeDef drv8234_read_reg(drv8234_t *dev, uint8_t reg, uint8_t *value);

/* */
HAL_StatusTypeDef drv8234_set_enable(drv8234_t *dev, uint8_t enable);
/**
 * Initialize DRV8234 with recommended configuration values.
 * Returns HAL_OK on success or the first HAL error encountered.
 */
HAL_StatusTypeDef drv8234_init(drv8234_t *dev);

#define DRV8234_ENABLE_BIT   0x80 /* bit7: EN_OUT */

/* Register addresses (Table 7-25: I2C Registers) */
#define DRV8234_REG_FAULT        0x00
#define DRV8234_REG_RC_STATUS1   0x01
#define DRV8234_REG_RC_STATUS2   0x02
#define DRV8234_REG_RC_STATUS3   0x03
#define DRV8234_REG_STATUS1      0x04
#define DRV8234_REG_STATUS2      0x05
#define DRV8234_REG_STATUS3      0x06

#define DRV8234_REG_CONFIG0      0x09
#define DRV8234_REG_CONFIG1      0x0A
#define DRV8234_REG_CONFIG2      0x0B
#define DRV8234_REG_CONFIG3      0x0C
#define DRV8234_REG_CONFIG4      0x0D

#define DRV8234_REG_CTRL0        0x0E
#define DRV8234_REG_CTRL1        0x0F
#define DRV8234_REG_CTRL2        0x10

#define DRV8234_REG_RC_CTRL0     0x11
#define DRV8234_REG_RC_CTRL1     0x12
#define DRV8234_REG_RC_CTRL2     0x13
#define DRV8234_REG_RC_CTRL3     0x14
#define DRV8234_REG_RC_CTRL4     0x15
#define DRV8234_REG_RC_CTRL5     0x16
#define DRV8234_REG_RC_CTRL6     0x17
#define DRV8234_REG_RC_CTRL7     0x18
#define DRV8234_REG_RC_CTRL8     0x19

/* FAULT (0x00) bits (left-to-right mapping -> bits7..0 as in table) */
#define DRV8234_FAULT_FAULT7     0x80
#define DRV8234_FAULT_FAULT6     0x40
#define DRV8234_FAULT_RSVD       0x20
#define DRV8234_FAULT_STALL      0x10
#define DRV8234_FAULT_OCP        0x08
#define DRV8234_FAULT_OVP        0x04
#define DRV8234_FAULT_TSD        0x02
#define DRV8234_FAULT_NPOR       0x01

/* RC_STATUS1 (0x01): SPEED[7:0] */
#define DRV8234_SPEED_MASK       0xFF

/* RC_STATUS2/3 (0x02/0x03): RC_CNT[15:0] */
#define DRV8234_RC_CNT_LO_MASK   0xFF
#define DRV8234_RC_CNT_HI_MASK   0xFF

/* REG_STATUS3 (0x06): IN_DUTY[5:0] in bits[5:0] */
#define DRV8234_IN_DUTY_MASK     0x3F

/* CONFIG0 (0x09) bits */
#define DRV8234_CFG0_EN_OUT      0x80 /* bit7 */
#define DRV8234_CFG0_EN_OVP      0x40 /* bit6 */
#define DRV8234_CFG0_EN_STALL    0x20 /* bit5 */
#define DRV8234_CFG0_VSNS_SEL    0x10 /* bit4 */
#define DRV8234_CFG0_RSVD        0x08 /* bit3 */
#define DRV8234_CFG0_CLR_CNT     0x04 /* bit2 */
#define DRV8234_CFG0_CLR_FLT     0x02 /* bit1 */
#define DRV8234_CFG0_DUTY_CTRL   0x01 /* bit0 */

/* CONFIG1/2 TINRUSH (0x0A/0x0B) */
#define DRV8234_TINRUSH_LO_MASK  0xFF
#define DRV8234_TINRUSH_HI_MASK  0xFF

/* CONFIG3 (0x0C) bits: IMODE[1:0], SMODE, INT_VREF, TBLANK, TDEG, OCP_MODE, TSD_MODE */
#define DRV8234_CFG3_IMODE_MASK  0xC0 /* bits7:6 */
#define DRV8234_CFG3_IMODE_SHIFT 6
#define DRV8234_CFG3_SMODE       0x20
#define DRV8234_CFG3_INT_VREF    0x10
#define DRV8234_CFG3_TBLANK      0x08
#define DRV8234_CFG3_TDEG        0x04
#define DRV8234_CFG3_OCP_MODE    0x02
#define DRV8234_CFG3_TSD_MODE    0x01

/* CONFIG4 (0x0D) bits: RC_REP[1:0], STALL_REP, CBC_REP, PMODE, I2C_BC, I2C_EN_IN1, I2C_PH_IN2 */
#define DRV8234_CFG4_RC_REP_MASK 0xC0
#define DRV8234_CFG4_RC_REP_SHIFT 6
#define DRV8234_CFG4_STALL_REP  0x20
#define DRV8234_CFG4_CBC_REP    0x10
#define DRV8234_CFG4_PMODE      0x08
#define DRV8234_CFG4_I2C_BC     0x04
#define DRV8234_CFG4_I2C_EN_IN1 0x02
#define DRV8234_CFG4_I2C_PH_IN2 0x01

/* REG_CTRL0 (0x0E): RSVD, EN_SS, REG_CTRL[1:0], PWM_FREQ, W_SCALE[1:0] */
#define DRV8234_CTRL0_EN_SS     0x40
#define DRV8234_CTRL0_REG_CTRL_MASK 0x30
#define DRV8234_CTRL0_REG_CTRL_SHIFT 4
#define DRV8234_CTRL0_PWM_FREQ  0x08
#define DRV8234_CTRL0_W_SCALE_MASK 0x03

/* REG_CTRL1 (0x0F): WSET_VSET[7:0] */
#define DRV8234_WSET_VSET_MASK  0xFF

/* REG_CTRL2 (0x10): OUT_FLT[1:0] (bits7:6), EXT_DUTY[5:0] (bits5:0) */
#define DRV8234_CTRL2_OUT_FLT_MASK 0xC0
#define DRV8234_CTRL2_OUT_FLT_SHIFT 6
#define DRV8234_CTRL2_EXT_DUTY_MASK 0x3F

/* RC_CTRL0 (0x11): EN_RC, DIS_EC, RC_HIZ, FLT_GAIN_SEL[1:0], CS_GAIN_SEL[2:0] */
#define DRV8234_RC0_EN_RC       0x80
#define DRV8234_RC0_DIS_EC      0x40
#define DRV8234_RC0_RC_HIZ      0x20
#define DRV8234_RC0_FLT_GAIN_MASK 0x18
#define DRV8234_RC0_FLT_GAIN_SHIFT 3
#define DRV8234_RC0_CS_GAIN_MASK 0x07

/* RC_CTRL1 (0x12): RC_THR[7:0] */
#define DRV8234_RC1_RC_THR_MASK  0xFF

/* RC_CTRL2 (0x13): INV_R_SCALE[1:0] (bits7:6), KMC_SCALE[1:0] (bits5:4),
   RC_THR_SCALE[1:0] (bits3:2), RC_THR[9:8] (bits1:0) */
#define DRV8234_RC2_INV_R_SCALE_MASK 0xC0
#define DRV8234_RC2_INV_R_SCALE_SHIFT 6
#define DRV8234_RC2_KMC_SCALE_MASK 0x30
#define DRV8234_RC2_KMC_SCALE_SHIFT 4
#define DRV8234_RC2_RC_THR_SCALE_MASK 0x0C
#define DRV8234_RC2_RC_THR_SCALE_SHIFT 2
#define DRV8234_RC2_RC_THR_HIGH_MASK 0x03

/* RC_CTRL3..RC_CTRL6 (0x14..0x16) */
#define DRV8234_RC3_INV_R_MASK   0xFF
#define DRV8234_RC4_KMC_MASK     0xFF
#define DRV8234_RC5_FLT_K_MASK   0x0F

/* RC_CTRL6 (0x17): EC_PULSE_DIS, T_MECH_FLT, EC_FALSE_PER, EC_MISS_PER */
#define DRV8234_RC6_EC_PULSE_DIS 0x80
#define DRV8234_RC6_T_MECH_FLT   0x40
#define DRV8234_RC6_EC_FALSE_PER_MASK 0x38 /* bits5:3 */
#define DRV8234_RC6_EC_MISS_PER_MASK  0x07 /* bits2:0 */

/* RC_CTRL7 (0x18): KP_DIV[2:0] (bits7:5), KP[4:0] (bits4:0) */
#define DRV8234_RC7_KP_DIV_MASK  0xE0
#define DRV8234_RC7_KP_DIV_SHIFT 5
#define DRV8234_RC7_KP_MASK      0x1F

/* RC_CTRL8 (0x19): KI_DIV[2:0] (bits7:5), KI[4:0] (bits4:0) */
#define DRV8234_RC8_KI_DIV_MASK  0xE0
#define DRV8234_RC8_KI_DIV_SHIFT 5
#define DRV8234_RC8_KI_MASK      0x1F


#ifdef __cplusplus
}
#endif

#endif /* __DRV8234_H */
