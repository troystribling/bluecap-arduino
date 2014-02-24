#include <SPI.h>
#include <EEPROM.h>
#include "boards.h"
#include "lib_aci.h"
#include "aci_setup.h"
#include "utils.h"

#include "blue_cap_peripheral.h"

BlueCapPeripheral::BlueCapBond::BlueCapBond() {
}

void BlueCapPeripheral::BlueCapBond::init(BlueCapPeripheral* _peripheral, uint16_t _eepromOffset, uint16_t _maxBonds, uint8_t _index) {
  eepromOffset = _eepromOffset;
  index = _index;
  maxBonds = _maxBonds;
  newBond = false;
  peripheral = _peripheral;
  if (status() == 0x00) {
    bonded = false;
  } else {
    bonded = true;
  }
}

void  BlueCapPeripheral::BlueCapBond::clearBondData() {
    EEPROM.write(offset(), 0x00);
}

void  BlueCapPeripheral::BlueCapBond::setup(aci_state_t* aciState) {
  aciState->bonded = ACI_BOND_STATUS_FAILED;
}

bool BlueCapPeripheral::BlueCapBond::restoreIfBonded(aci_state_t* aciState) {
  bool result = true;
  if (bonded) {
    DLOG(F("Previous Bond present. Restoring"));
    if (ACI_STATUS_TRANSACTION_COMPLETE == restoreBondData(aciState)) {
      DLOG(F("Bond restored successfully: Waiting for connection"));
      // prevents HW error
      delay(1000);
    }
    else {
      ERROR(F("Bond restore failed. Delete the bond and try again."));
      result = false;
    }
  }
  return result;
}

void BlueCapPeripheral::BlueCapBond::writeIfBonded(aci_state_t* aciState, aci_evt_t* aciEvt) {
  if (ACI_BOND_STATUS_SUCCESS == aciState->bonded) {
    DLOG(F("ACI_BOND_STATUS_SUCCESS"));
    aciState->bonded = ACI_BOND_STATUS_FAILED;
    if (ACI_STATUS_EXTENDED == aciEvt->params.disconnected.aci_status) {
      if (!bonded) {
        if (readAndWriteBondData(aciState)) {
          bonded = true;
          DLOG(F("Bond data read and store successful"));
          // prevents HW error
          delay(1000);
        } else {
          ERROR(F("Bond data read and store failed"));
        }
      }
    }
  }
}

// private
aci_status_code_t  BlueCapPeripheral::BlueCapBond::restoreBondData(aci_state_t* aciState) {
  aci_evt_t *aciEvt;
  uint16_t addr = readBondDataOffset();
  uint8_t numDynMsgs = status() & 0x7F;
  hal_aci_data_t aciCmd;

  while(1) {

    addr = readBondData(&aciCmd, addr);

    if (!hal_aci_tl_send(&aciCmd)) {
      ERROR(F("restoreBondData: failed"));
      return ACI_STATUS_ERROR_INTERNAL;
    }

    while (1) {
      if (lib_aci_event_get(aciState, &aciData)) {
        aciEvt = &aciData.evt;
        if (ACI_EVT_CMD_RSP != aciEvt->evt_opcode) {
            ERROR(F("restoreBondData: failed with error: 0x"));
            ERROR(aciEvt->evt_opcode, HEX);
            return ACI_STATUS_ERROR_INTERNAL;
        } else {
          if (ACI_STATUS_TRANSACTION_COMPLETE == aciEvt->params.cmd_rsp.cmd_status) {
            bonded = true;
            aciState->bonded = ACI_BOND_STATUS_SUCCESS;
            DLOG(F("Restore of bond data completed successfully"));
            return ACI_STATUS_TRANSACTION_COMPLETE;
          } else if (ACI_STATUS_TRANSACTION_CONTINUE == aciEvt->params.cmd_rsp.cmd_status) {
            numDynMsgs--;
            break;
          } else if (0 >= numDynMsgs) {
            ERROR(F("Restore of bond data failed with too many messages"));
            return ACI_STATUS_ERROR_INTERNAL;
          } else {
            ERROR(F("Restore of bond data failed with cmd_status:"));
            ERROR(aciEvt->params.cmd_rsp.cmd_status, HEX);
            return ACI_STATUS_ERROR_INTERNAL;
          }
        }
      }
    }
  }
}

bool  BlueCapPeripheral::BlueCapBond::readAndWriteBondData(aci_state_t* aciState) {
  bool status = false;
  aci_evt_t* aciEvt = NULL;
  uint8_t numDynMsgs = 0;
  uint16_t addr = readBondDataOffset();

  lib_aci_read_dynamic_data();
  numDynMsgs++;

  while (1) {
    if (lib_aci_event_get(aciState, &aciData)) {
      aciEvt = &aciData.evt;
      if (ACI_EVT_CMD_RSP != aciEvt->evt_opcode ) {
        ERROR(F("readAndWriteBondData command response failed:"));
        ERROR(aciEvt->evt_opcode, HEX);
        status = false;
        break;
      } else if (ACI_STATUS_TRANSACTION_COMPLETE == aciEvt->params.cmd_rsp.cmd_status) {
        addr = writeBondData(aciEvt, addr);
        writeBondDataHeader(addr, numDynMsgs);
        status = true;
        break;
      } else if (!(ACI_STATUS_TRANSACTION_CONTINUE == aciEvt->params.cmd_rsp.cmd_status)) {
        ERROR(F("readAndWriteBondData transaction failed:"));
        ERROR(aciEvt->params.cmd_rsp.cmd_status, HEX);
        clearBondData();
        status = false;
        break;
      } else {
        addr = writeBondData(aciEvt, addr);
        lib_aci_read_dynamic_data();
        numDynMsgs++;
      }
    }
  }
  return status;
}

uint16_t  BlueCapPeripheral::BlueCapBond::writeBondData(aci_evt_t* evt, uint16_t addr) {
  EEPROM.write(addr, evt->len - 2);
  addr++;
  EEPROM.write(addr, ACI_CMD_WRITE_DYNAMIC_DATA);
  addr++;
  for (uint8_t i=0; i< (evt->len-3); i++) {
    EEPROM.write(addr, evt->params.cmd_rsp.params.padding[i]);
    addr++;
  }
  return addr;
}

uint16_t BlueCapPeripheral::BlueCapBond::readBondData(hal_aci_data_t* aciCmd, uint16_t addr) {
  uint8_t len = EEPROM.read(addr);
  addr++;
  aciCmd->buffer[0] = len;
  for (uint8_t i = 1; i <= len; i++) {
      aciCmd->buffer[i] = EEPROM.read(addr);
      addr++;
  }
  return addr;
}

void BlueCapPeripheral::BlueCapBond::connectOrBond() {
  if (bonded) {
    peripheral->connect();
    DLOG(F("Advertising started. Waiting for connection with bond:"));
  } else {
    peripheral->bond();
    DLOG(F("Advertising started : Waiting for connection and bonding with bond:"));
  }
  DLOG(index, DEC);
}

void BlueCapPeripheral::BlueCapBond::writeBondDataHeader(uint16_t dataAddress, uint8_t numDynMsgs) {
  uint8_t dataSize = dataAddress - readBondDataOffset();
  EEPROM.write(offset(), 0x80 | numDynMsgs);
  EEPROM.write(offset() + 1, dataSize);
}

uint16_t BlueCapPeripheral::BlueCapBond::readBondDataOffset() {
  uint16_t bondOffset = eepromOffset + maxBonds*BOND_HEADER_BYTES;
  for (int i = 0; i < index; i++) {
    bondOffset += EEPROM.read(eepromOffset + i*BOND_HEADER_BYTES + 1);
  }
  return bondOffset;
}

uint8_t  BlueCapPeripheral::BlueCapBond::status() {
  return EEPROM.read(offset());
}

uint16_t BlueCapPeripheral::BlueCapBond::offset() {
  return eepromOffset + index*BOND_HEADER_BYTES;
}