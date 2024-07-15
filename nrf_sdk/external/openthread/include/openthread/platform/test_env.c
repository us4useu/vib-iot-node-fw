result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true);

ICM_20948_Status_e ICM_20948::i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
  status = ICM_20948_i2c_controller_configure_peripheral(&_device, peripheral, addr, reg, len, Rw, enable, data_only, grp, swap, dataOut);
  return status;
}

ICM_20948_Status_e ICM_20948_i2c_controller_configure_peripheral(ICM_20948_Device_t *pdev, uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  uint8_t periph_addr_reg;
  uint8_t periph_reg_reg;
  uint8_t periph_ctrl_reg;
  uint8_t periph_do_reg;

  switch (peripheral)
  {
  case 0:
    periph_addr_reg = AGB3_REG_I2C_PERIPH0_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH0_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH0_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH0_DO;
    break;
  case 1:
    periph_addr_reg = AGB3_REG_I2C_PERIPH1_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH1_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH1_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH1_DO;
    break;
  case 2:
    periph_addr_reg = AGB3_REG_I2C_PERIPH2_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH2_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH2_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH2_DO;
    break;
  case 3:
    periph_addr_reg = AGB3_REG_I2C_PERIPH3_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH3_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH3_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH3_DO;
    break;
  default:
    return ICM_20948_Stat_ParamErr;
  }

  retval = ICM_20948_set_bank(pdev, 3);
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  // Set the peripheral address and the Rw flag
  ICM_20948_I2C_PERIPHX_ADDR_t address;
  address.ID = addr;
  if (Rw)
  {
    address.RNW = 1;
  }
  else
  {
    address.RNW = 0; // Make sure bit is clear (just in case there is any garbage in that RAM location)
  }
  retval = ICM_20948_execute_w(pdev, periph_addr_reg, (uint8_t *)&address, sizeof(ICM_20948_I2C_PERIPHX_ADDR_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  // If we are setting up a write, configure the Data Out register too
  if (!Rw)
  {
    ICM_20948_I2C_PERIPHX_DO_t dataOutByte;
    dataOutByte.DO = dataOut;
    retval = ICM_20948_execute_w(pdev, periph_do_reg, (uint8_t *)&dataOutByte, sizeof(ICM_20948_I2C_PERIPHX_DO_t));
    if (retval != ICM_20948_Stat_Ok)
    {
      return retval;
    }
  }

  // Set the peripheral sub-address (register address)
  ICM_20948_I2C_PERIPHX_REG_t subaddress;
  subaddress.REG = reg;
  retval = ICM_20948_execute_w(pdev, periph_reg_reg, (uint8_t *)&subaddress, sizeof(ICM_20948_I2C_PERIPHX_REG_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  // Set up the control info
  ICM_20948_I2C_PERIPHX_CTRL_t ctrl;
  ctrl.LENG = len;
  ctrl.EN = enable;
  ctrl.REG_DIS = data_only;
  ctrl.GRP = grp;
  ctrl.BYTE_SW = swap;
  retval = ICM_20948_execute_w(pdev, periph_ctrl_reg, (uint8_t *)&ctrl, sizeof(ICM_20948_I2C_PERIPHX_CTRL_t));
  if (retval != ICM_20948_Stat_Ok)
  {
    return retval;
  }

  return retval;
}