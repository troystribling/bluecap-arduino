#include <SPI.h>
#include <EEPROM.h>
#include "nordic/boards.h"
#include "nordic/lib_aci.h"
#include "nordic/aci_setup.h"
#include "dlog.h"
#include "blue_cap_bond.h"

BlueCapBond::BlueCapBond() {
  init(0);
}

BlueCapBond::BlueCapBond(uint16_t _eepromOffset) {
  init(_eepromOffset);
}

void BlueCapBond::clearBondData() {
    EEPROM.write(eepromOffset, 0x00);
}

uint8_t BlueCapBond::status() {
  return EEPROM.read(eepromOffset);
}

void BlueCapBond::setup(aci_state_t* aciState) {
  DLOG(F("Initial eepromOffset:"));
  DLOG(eepromOffset, HEX);
  aciState->bonded = ACI_BOND_STATUS_FAILED;
}

uint16_t BlueCapBond::writeBondData(aci_evt_t* evt, uint16_t addr) {
  DLOG(F("writeBondData"));
  DLOG(F("Message length:"));
  DLOG(evt->len - 2, HEX);
  EEPROM.write(addr, evt->len - 2);
  addr++;
  DLOG(F("Event op_code:"));
  DLOG(ACI_CMD_WRITE_DYNAMIC_DATA, HEX);
  EEPROM.write(addr, ACI_CMD_WRITE_DYNAMIC_DATA);
  addr++;
  for (uint8_t i=0; i< (evt->len-3); i++) {
    DLOG(F("message address and value:"));
    DLOG(addr, HEX);
    DLOG(evt->params.cmd_rsp.params.padding[i], HEX);
    EEPROM.write(addr, evt->params.cmd_rsp.params.padding[i]);
    addr++;
  }
  return addr;
}

aci_status_code_t BlueCapBond::restoreBondData(uint8_t eepromStatus, aci_state_t* aciState) {
  aci_evt_t *aciEvt;
  uint16_t addr = eepromOffset + 1;
  uint8_t len = 0;
  uint8_t numDynMsgs = eepromStatus & 0x7F;

  DLOG(F("restoreBondData dynamic messages:"));
  DLOG(numDynMsgs, HEX);

  while(1) {

    DLOG(F("Restore messages:"));
    DLOG(numDynMsgs);

    len = EEPROM.read(addr);
    addr++;
    aciCmd.buffer[0] = len;

    DLOG(F("Message len:"));
    DLOG(len, HEX);

    DLOG(F("Start reading message"));
    for (uint8_t i = 1; i <= len; i++) {
        aciCmd.buffer[i] = EEPROM.read(addr);
        DLOG(F("message address and value:"));
        DLOG(addr, HEX);
        DLOG(aciCmd.buffer[i], HEX);
        addr++;
    }
    DLOG(F("Finished reading message"));

    DLOG(F("Send restore command"));
    if (!hal_aci_tl_send(&aciCmd)) {
      DLOG(F("restoreBondData: failed"));
      return ACI_STATUS_ERROR_INTERNAL;
    }

    while (1) {
      if (lib_aci_event_get(aciState, &aciData)) {
        aciEvt = &aciData.evt;
        DLOG(F("Event recieved with opcode:"));
        DLOG(aciEvt->evt_opcode, HEX);
        if (ACI_EVT_CMD_RSP != aciEvt->evt_opcode) {
            DLOG(F("restoreBondData: failed with error: 0x"));
            DLOG(aciEvt->evt_opcode, HEX);
            return ACI_STATUS_ERROR_INTERNAL;
        } else {
          numDynMsgs--;
          DLOG(F("Command status:"));
          DLOG(aciEvt->params.cmd_rsp.cmd_status, HEX);
          if (ACI_STATUS_TRANSACTION_COMPLETE == aciEvt->params.cmd_rsp.cmd_status) {
            bondedFirstTimeState = false;
            aciState->bonded = ACI_BOND_STATUS_SUCCESS;
            DLOG(F("Restore of bond data completed successfully"));
            return ACI_STATUS_TRANSACTION_COMPLETE;
          } else if (ACI_STATUS_TRANSACTION_CONTINUE == aciEvt->params.cmd_rsp.cmd_status) {
            DLOG(F("Restore next bond data message"));
            break;
          } else if (0 >= numDynMsgs) {
            DLOG(F("Restore of bond data failed with too many messages"));
            return ACI_STATUS_ERROR_INTERNAL;
          } else {
            DLOG(F("Restore of bond data failed with cmd_status:"));
            DLOG(aciEvt->params.cmd_rsp.cmd_status, HEX);
            return ACI_STATUS_ERROR_INTERNAL;
          }
        }
      }
    }
  }
}

bool BlueCapBond::readAndWriteBondData(aci_state_t* aciState) {
  bool status = false;
  aci_evt_t* aciEvt = NULL;
  uint8_t numDynMsgs = 0;
  uint8_t addr = eepromOffset + 1;

  lib_aci_read_dynamic_data();
  numDynMsgs++;

  DLOG(F("readAndWriteBondData"));

  while (1) {
    if (lib_aci_event_get(aciState, &aciData)) {
      DLOG(F("Recieved event for message:"));
      DLOG(numDynMsgs, DEC);
      aciEvt = &aciData.evt;
      if (ACI_EVT_CMD_RSP != aciEvt->evt_opcode ) {
        DLOG(F("readAndWriteBondData command response failed:"));
        DLOG(aciEvt->evt_opcode, HEX);
        status = false;
        break;
      } else if (ACI_STATUS_TRANSACTION_COMPLETE == aciEvt->params.cmd_rsp.cmd_status) {
        DLOG(F("readAndWriteBondData transaction complete, status, eepromOffset"));
        DLOG(0x80 | numDynMsgs, HEX);
        DLOG(eepromOffset, DEC);
        writeBondData(aciEvt, addr);
        EEPROM.write(eepromOffset, 0x80 | numDynMsgs);
        status = true;
        break;
      } else if (!(ACI_STATUS_TRANSACTION_CONTINUE == aciEvt->params.cmd_rsp.cmd_status)) {
        DLOG(F("readAndWriteBondData transaction failed:"));
        DLOG(aciEvt->params.cmd_rsp.cmd_status, HEX);
        EEPROM.write(eepromOffset, 0x00);
        status = false;
        break;
      } else {
        DLOG(F("readAndWriteBondData transaction continue"));
        addr = writeBondData(aciEvt, addr);
        lib_aci_read_dynamic_data();
        numDynMsgs++;
      }
    }
  }
  return status;
}

// private
void BlueCapBond::init(uint16_t _eepromOffset) {
  eepromOffset = _eepromOffset;
  bondedFirstTimeState = true;
}