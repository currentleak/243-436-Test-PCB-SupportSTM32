#include "drv8234.h"

HAL_StatusTypeDef drv8234_write_reg(drv8234_t *dev, uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  for (int i = 0; i < 3; ++i) {
    ret = HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->address << 1), (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
    if (ret == HAL_OK) return HAL_OK;
    HAL_Delay(5);
  }
  return ret;
}

HAL_StatusTypeDef drv8234_read_reg(drv8234_t *dev, uint8_t reg, uint8_t *value)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  for (int i = 0; i < 3; ++i) {
    ret = HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->address << 1), (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, value, 1, 100);
    if (ret == HAL_OK) return HAL_OK;
    HAL_Delay(5);
  }
  return ret;
}

HAL_StatusTypeDef drv8234_set_enable(drv8234_t *dev, uint8_t enable)
{
  uint8_t val = 0;
  HAL_StatusTypeDef st = drv8234_read_reg(dev, DRV8234_REG_CONFIG0, &val);
  if (st != HAL_OK) {
    /* If read fails, try writing the enable bit directly */
    val = enable ? DRV8234_ENABLE_BIT : 0;
    return drv8234_write_reg(dev, DRV8234_REG_CONFIG0, val);
  }

  if (enable)
    val |= DRV8234_ENABLE_BIT;
  else
    val &= ~DRV8234_ENABLE_BIT;

  return drv8234_write_reg(dev, DRV8234_REG_CONFIG0, val);
}

HAL_StatusTypeDef drv8234_init(drv8234_t *dev)
{
  HAL_StatusTypeDef st = HAL_OK;

  /* CONFIG0 = 0x60: Outputs disabled (Hi-Z), OVP and stall detection enabled */
  st = drv8234_write_reg(dev, DRV8234_REG_CONFIG0, 0x60);
  if (st != HAL_OK) return st;

  /* CONFIG3 = 0x63: IMODE=01b, SMODE=1, OCP_MODE=1, TSD_MODE=1 */
  st = drv8234_write_reg(dev, DRV8234_REG_CONFIG3, 0x63);
  if (st != HAL_OK) return st;

  /* CONFIG4 = 0xB8: RC_REP=[nFAULT indicates RC_CNT done], PWM Mode */
  st = drv8234_write_reg(dev, DRV8234_REG_CONFIG4, 0xB8);
  if (st != HAL_OK) return st;

  /* REG_CTRL0 = 0x24: EN_SS=1, Fixed Off-Time Current Regulation, 25kHz, W_SCALE=16 */
  st = drv8234_write_reg(dev, DRV8234_REG_CTRL0, 0x24);
  if (st != HAL_OK) return st;

  /* REG_CTRL1 = 0xFF: WSET_VSET=0xFF */
  st = drv8234_write_reg(dev, DRV8234_REG_CTRL1, 0xFF);
  if (st != HAL_OK) return st;

  /* REG_CTRL2 = 0x00: OUT_FLT=250Hz (Recommended) */
  st = drv8234_write_reg(dev, DRV8234_REG_CTRL2, 0x00);
  if (st != HAL_OK) return st;

  /* RC_CTRL0 = 0xFB: EN_RC=1, DIS_EC=1 (Error Correction DISABLED), RC_HIZ=1 (Disable output after # Ripples), */
  st = drv8234_write_reg(dev, DRV8234_REG_RC_CTRL0, 0xFB);
  if (st != HAL_OK) return st;

  /* RC_CTRL3 = 0x2F: INV_R =INV_RSCALE/motor_res */
  st = drv8234_write_reg(dev, DRV8234_REG_RC_CTRL3, 0x2F);
  if (st != HAL_OK) return st;

  /* RC_CTRL4 = 0xF8: KMC=248 */
  st = drv8234_write_reg(dev, DRV8234_REG_RC_CTRL4, 0xF8);
  if (st != HAL_OK) return st;

  /* RC_CTRL5 = 0x60: FLT_K=0.5, sets the BW of the BP filter (recommended value 6) */
  st = drv8234_write_reg(dev, DRV8234_REG_RC_CTRL5, 0x60);
  if (st != HAL_OK) return st;

  /* RC_CTRL6 = 0x65: T_MECH_FLT */
  st = drv8234_write_reg(dev, DRV8234_REG_RC_CTRL6, 0x65);
  if (st != HAL_OK) return st;

  // Use RC_THR * RC_THR_SCALE to set how many ripples to move to until it stops. 
  /* RC_CTRL1 = 0x19: RC_THR */
  st = drv8234_write_reg(dev, DRV8234_REG_RC_CTRL1, 0x19);
  if (st != HAL_OK) return st;

  /* RC_CTRL2 = 0xA4: INV_R_SCALE/KMC_SCALE/RC_THR_SCALE */
  st = drv8234_write_reg(dev, DRV8234_REG_RC_CTRL2, 0xA4);
  return st;
}
