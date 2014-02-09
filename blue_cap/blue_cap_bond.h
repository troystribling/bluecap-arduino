#ifndef _BLUE_CAP_BOND_H
#define _BLUE_CAP_BOND_H

#include "nordic/lib_aci.h"

class BlueCapBond {

public:

  BlueCapBond();
  BlueCapBond(uint16_t _eepromOffset);

  void clearBondData();
  uint8_t status();
  void setup(aci_state_t* aciState);

  uint16_t writeBondData(aci_evt_t* evt, uint16_t addr);
  aci_status_code_t restoreBondData(uint8_t eepromStatus, aci_state_t* aciState);
  bool readAndWriteBondData(aci_state_t* aciState);

  bool                            bondedFirstTimeState;

private:

  uint16_t                        eepromOffset;
  hal_aci_data_t                  aciCmd;
  hal_aci_evt_t                   aciData;

private:

  void init(uint16_t _eepromOffset);

};

#endif
