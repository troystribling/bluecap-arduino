#include <SPI.h>
#include <EEPROM.h>
#include "nordic/boards.h"
#include "nordic/lib_aci.h"
#include "nordic/aci_setup.h"
#include "dlog.h"

#include "blue_cap_peripheral.h"

// public methods
BlueCapPeripheral::BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin) {
	init(_reqnPin, _rdynPin, false, 0);
}

BlueCapPeripheral::BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin, uint16_t _eepromOffset) {
	init(_reqnPin, _rdynPin, false, _eepromOffset);
}

BlueCapPeripheral::BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin, uint16_t _eepromOffset, bool _bond) {
	init(_reqnPin, _rdynPin, _bond, _eepromOffset);
}

BlueCapPeripheral::~BlueCapPeripheral() {
}

bool BlueCapPeripheral::sendAck(uint8_t pipe) {
	bool status = false;
	if (isPipeAvailable(pipe)) {
		waitForCredit();
		status = lib_aci_send_ack(&aciState, pipe);
	}
	if (status) {
		waitForAck();
		DLOG(F("ACK successful over pipe:"));
  } else {
    DLOG(F("ACK failed over pipe:"));
 }
	DLOG(pipe, HEX);
	return status;
}

bool BlueCapPeripheral::sendNack(uint8_t pipe, const uint8_t errorCode) {
	bool status = false;
	if (isPipeAvailable(pipe)) {
		waitForCredit();
		status = lib_aci_send_nack(&aciState, pipe, errorCode);
	}
	if (status) {
		waitForAck();
		DLOG(F("NACK successful over pipe:"));
  } else {
    DLOG(F("NACK failed over pipe:"));
  }
	DLOG(pipe, HEX);
	return status;
}

bool BlueCapPeripheral::sendData(uint8_t pipe, uint8_t* value, uint8_t size) {
	bool status = false;
	if (isPipeAvailable(pipe)) {
		waitForCredit();
		status = lib_aci_send_data(pipe, value, size);
	}
	if (status) {
		waitForAck();
		DLOG(F("sendData successful over pipe:"));
	} else {
		DLOG(F("sendData failed over pipe:"));
	}
		DLOG(pipe, HEX);
	return status;
}

bool BlueCapPeripheral::requestData(uint8_t pipe) {
	waitForCredit();
	bool status = true;
	if (isPipeAvailable(pipe)) {
		status = lib_aci_request_data(&aciState, pipe);
	}
	if (status) {
		waitForAck();
		DLOG(F("requestData successful over pipe:"));
	} else {
		DLOG(F("requestData failed over pipe:"));
	}
	DLOG(pipe, HEX);
	return status;
}

bool BlueCapPeripheral::setData(uint8_t pipe, uint8_t* value, uint8_t size) {
	bool status = lib_aci_set_local_data(&aciState, pipe, value, size);
	if (status) {
		DLOG(F("setData successful over pipe:"));
	} else {
		DLOG(F("sendData failed over pipe:"));
	}
	DLOG(pipe, HEX);
	return status;
}

bool BlueCapPeripheral::setTxPower(aci_device_output_power_t txPower) {
	waitForCmdComplete();
	bool status = lib_aci_set_tx_power(txPower);
	if (status) {
		DLOG(F("setTxPower successful"));
	} else {
		DLOG(F("setTxPower failed"));
		cmdComplete = true;
	}
	return status;
}

bool BlueCapPeripheral::getBatteryLevel() {
	waitForCmdComplete();
	bool status = lib_aci_get_battery_level();
	if (status) {
		DLOG(F("getBatteryLevel successful"));
	} else {
		DLOG(F("getBatteryLevel failed"));
		cmdComplete = true;
	}
	return status;
}

bool BlueCapPeripheral::getTemperature() {
	waitForCmdComplete();
	bool status = lib_aci_get_temperature();
	if (status) {
		DLOG(F("getTemperartue successful"));
	} else {
		DLOG(F("getTemperartue failed"));
		cmdComplete = true;
	}
	return status;
}

bool BlueCapPeripheral::getDeviceVersion() {
	waitForCmdComplete();
	bool status = lib_aci_device_version();
	if (status) {
		DLOG(F("getDeviceVersion successful"));
	} else {
		DLOG(F("getDeviceVersion failed"));
		cmdComplete = true;
	}
	return status;
}

bool BlueCapPeripheral::getAddress() {
	waitForCmdComplete();
	bool status = lib_aci_get_address();
	if (status) {
		DLOG(F("getAddress successful"));
	} else {
		DLOG(F("getAddress failed"));
		cmdComplete = true;
	}
	return status;
}

void BlueCapPeripheral::clearBondData() {
    EEPROM.write(eepromOffset, 0x00);
}

// protected
void BlueCapPeripheral::setServicePipeTypeMapping(services_pipe_type_mapping_t* mapping, int count) {
	servicesPipeTypeMapping = mapping;
	numberOfPipes = count;
}

void BlueCapPeripheral::setSetUpMessages(hal_aci_data_t* messages, int count) {
	setUpMessages = messages;
	numberOfSetupMessages = count;
}

bool BlueCapPeripheral::isPipeAvailable(uint8_t pipe) {
	bool status = lib_aci_is_pipe_available(&aciState, pipe);
	if (status) {
		DLOG(F("Pipe available:"));
	} else {
		DLOG(F("Pipe unavailable:"));
	}
	DLOG(pipe, HEX);
	return status;
}

// private methods
void BlueCapPeripheral::init(uint8_t _reqnPin, uint8_t _rdynPin, bool _bond, uint16_t _eepromOffset) {
	setUpMessages = NULL;
	numberOfSetupMessages = 0;
	servicesPipeTypeMapping = NULL;
	numberOfPipes = 0;
	isConnected = false;
	ack = false;
	reqnPin = _reqnPin;
	rdynPin = _rdynPin;
	bond = _bond;
	timingChangeDone = false;
	cmdComplete = true;
	eepromOffset = _eepromOffset;
	bondedFirstTimeState = true;
}

void BlueCapPeripheral::listen() {
	if (lib_aci_event_get(&aciState, &aciData)) {
		aci_evt_t  *aciEvt;
		aciEvt = &aciData.evt;
		switch(aciEvt->evt_opcode) {
			case ACI_EVT_DEVICE_STARTED:
				aciState.data_credit_total = aciEvt->params.device_started.credit_available;
				DLOG(F("Total credits"));
				DLOG(aciState.data_credit_total, DEC);
				switch(aciEvt->params.device_started.device_mode) {
					case ACI_DEVICE_SETUP:
						DLOG(F("ACI_DEVICE_SETUP"));
						if (ACI_STATUS_TRANSACTION_COMPLETE != do_aci_setup(&aciState)) {
							DLOG(F("Error ACI_DEVICE_SETUP"));
						}
						break;
					case ACI_DEVICE_STANDBY: {
						DLOG(F("ACI_DEVICE_STANDBY"));
						if (bond) {
					    uint8_t eepromStatus = 0;
              eepromStatus = EEPROM.read(eepromOffset);
              DLOG(F("eepromStatus:"));
              DLOG(eepromStatus);
              if (eepromStatus != 0x00) {
                DLOG(F("Previous Bond present. Restoring"));
                if (ACI_STATUS_TRANSACTION_COMPLETE == restoreBondData(eepromStatus)) {
                  DLOG(F("Bond restored successfully"));
                  lib_aci_connect(100/* in seconds */, 0x0020 /* advertising interval 20ms*/);
									didStartAdvertising();
									DLOG(F("Advertising started"));
                }
                else {
                  DLOG(F("Bond restore failed. Delete the bond and try again."));
                }
              }
              if (ACI_BOND_STATUS_SUCCESS != aciState.bonded) {
	              lib_aci_bond(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
								didStartAdvertising();
	              DLOG(F("Advertising started : Waiting to be connected and bonded"));
	            } else {
                lib_aci_connect(100/* in seconds */, 0x0020 /* advertising interval 20ms*/);
                Serial.println(F("Already bonded : Advertising started : Waiting to be connected"));
	            }
						} else {
              DLOG(F("No Bond present in EEPROM."));
							lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
							didStartAdvertising();
							DLOG(F("Advertising started"));
						}
						break;
					}
				}
				break;

			case ACI_EVT_CMD_RSP:
				DLOG(F("ACI_EVT_CMD_RSP"));
				DLOG(aciEvt->params.cmd_rsp.cmd_opcode, HEX);
				if (ACI_STATUS_SUCCESS != aciEvt->params.cmd_rsp.cmd_status) {
					DLOG(F("ACI_EVT_CMD_RSP: Error. Arduino is in an while(1); loop"));
					while(1){delay(1000);};
				} else {
					didReceiveCommandResponse(aciEvt->params.cmd_rsp.cmd_opcode,
						aciEvt->params.data_received.rx_data.aci_data, aciEvt->len - 3);
				}
				cmdComplete = true;
				break;

			case ACI_EVT_CONNECTED:
				DLOG(F("ACI_EVT_CONNECTED"));
				isConnected = true;
				timingChangeDone = false;
				aciState.data_credit_available = aciState.data_credit_total;
				didConnect();
				lib_aci_device_version();
				break;

      case ACI_EVT_BOND_STATUS:
				DLOG(F("ACI_EVT_BOND_STATUS"));
        aciState.bonded = aciEvt->params.bond_status.status_code;
				DLOG(aciState.bonded, HEX);
				if (aciState.bonded == ACI_BOND_STATUS_SUCCESS) {
					DLOG(F("Bond successful"));
					didBond();
				} else {
					DLOG(F("Bond failed"));
				}
        break;

			case ACI_EVT_PIPE_STATUS:
				DLOG(F("ACI_EVT_PIPE_STATUS"));
				didReceiveStatusChange();
				if (doTimingChange() && (timingChangeDone == false)) {
					lib_aci_change_timing_GAP_PPCP();
					timingChangeDone = true;
				}
				break;

			case ACI_EVT_TIMING:
				DLOG(F("ACI_EVT_TIMING"));
				break;

			case ACI_EVT_DISCONNECTED:
				isConnected = false;
				ack = true;
				DLOG(F("ACI_EVT_DISCONNECTED"));
				didDisconnect();
				if (bond) {
					if (ACI_BOND_STATUS_SUCCESS == aciState.bonded) {
						if (ACI_STATUS_EXTENDED == aciEvt->params.disconnected.aci_status) {
							if (bondedFirstTimeState) {
								bondedFirstTimeState = false;
								if (readAndWriteBondData()) {
									DLOG(F("Bond data read and store successful"));
								}
							}
							if (0x24 == aciEvt->params.disconnected.btle_status) {
								DLOG(F("Central deleted bond data"));
							}
						}
	          lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
	          DLOG(F("Using existing bond stored in EEPROM."));
	          DLOG(F("Advertising started. Connecting."));
					} else {
					  lib_aci_bond(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
						didStartAdvertising();
            DLOG(F("Advertising started : Waiting to be connected and bonded"));
					}
				} else {
					lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
					DLOG(F("Advertising started"));
					didStartAdvertising();
				}
				break;

			case ACI_EVT_DATA_RECEIVED: {
				int pipe = aciEvt->params.data_received.rx_data.pipe_number;
				int size = aciEvt->len - 2;
				ack = true;
				DLOG(F("ACI_EVT_DATA_RECEIVED Pipe #:"));
				DLOG(pipe, HEX);
				DLOG(F("size:"));
				DLOG(size, DEC);
				didReceiveData(pipe, aciEvt->params.data_received.rx_data.aci_data, size);
				break;
			}

			case ACI_EVT_DATA_CREDIT:
				aciState.data_credit_available = aciState.data_credit_available + aciEvt->params.data_credit.credit;
				DLOG(F("ACI_EVT_DATA_CREDIT"));
				DLOG(aciState.data_credit_available, DEC);
				ack = true;
				break;

			case ACI_EVT_PIPE_ERROR:
				ack = true;
				DLOG(F("ACI_EVT_PIPE_ERROR: Pipe #:"));
				DLOG(aciEvt->params.pipe_error.pipe_number, DEC);
				DLOG(F("Pipe Error Code: 0x"));
				DLOG(aciEvt->params.pipe_error.error_code, HEX);
				didReceiveError(aciEvt->params.pipe_error.pipe_number, aciEvt->params.pipe_error.error_code);
				incrementCredit();
				break;
		}
	}
}

void BlueCapPeripheral::setup() {
	aciState.aci_setup_info.services_pipe_type_mapping 	= servicesPipeTypeMapping;
	aciState.aci_setup_info.number_of_pipes    					= numberOfPipes;
	aciState.aci_setup_info.setup_msgs         					= setUpMessages;
	aciState.aci_setup_info.num_setup_msgs     					= numberOfSetupMessages;

	aciState.aci_pins.board_name = REDBEARLAB_SHIELD_V1_1;
	aciState.aci_pins.reqn_pin   = reqnPin;
	aciState.aci_pins.rdyn_pin   = rdynPin;
	aciState.aci_pins.mosi_pin   = MOSI;
	aciState.aci_pins.miso_pin   = MISO;
	aciState.aci_pins.sck_pin    = SCK;

#if defined(__SAM3X8E__)
	aciState.aci_pins.spi_clock_divider     = 84;
#else
	aciState.aci_pins.spi_clock_divider     = SPI_CLOCK_DIV8;
#endif

	aciState.aci_pins.reset_pin             = UNUSED;
	aciState.aci_pins.active_pin            = UNUSED;
	aciState.aci_pins.optional_chip_sel_pin = UNUSED;

	aciState.aci_pins.interface_is_interrupt	= false;
	aciState.aci_pins.interrupt_number			  = 1;

	lib_aci_init(&aciState);
	delay(100);

  DLOG(F("Initial eepromOffset:"));
  DLOG(eepromOffset, HEX);

	if (bond) {
		aciState.bonded = ACI_BOND_STATUS_FAILED;
	}
}

void BlueCapPeripheral::incrementCredit() {
	aciState.data_credit_available++;
	DLOG(F("Data Credit available:"));
	DLOG(aciState.data_credit_available,DEC);
}

void BlueCapPeripheral::decrementCredit() {
	aciState.data_credit_available--;
	DLOG(F("Data Credit available:"));
	DLOG(aciState.data_credit_available,DEC);
}

void BlueCapPeripheral::waitForCredit() {
	while(aciState.data_credit_available == 0){listen();}
}

void BlueCapPeripheral::waitForAck() {
		decrementCredit();
		ack = false;
		while(!ack){listen();}
}

void BlueCapPeripheral::waitForCmdComplete () {
	while(!cmdComplete){listen();};
	cmdComplete = false;
}

uint16_t BlueCapPeripheral::writeBondData(aci_evt_t* evt, uint16_t addr) {
	DLOG(F("writeBondData"));
  EEPROM.write(addr, evt->len - 2);
  EEPROM.write(addr, ACI_CMD_WRITE_DYNAMIC_DATA);
  addr++;
  for (uint8_t i=0; i< (evt->len-3); i++) {
  	DLOG(F("message address:"));
  	DLOG(addr, DEC);
    EEPROM.write(addr, evt->params.cmd_rsp.params.padding[i]);
    addr++;
  }
  return addr;
}

aci_status_code_t BlueCapPeripheral::restoreBondData(uint8_t eepromStatus) {
  aci_evt_t *aciEvt;
  uint16_t addr = eepromOffset + 1;
  uint8_t len = 0;
  uint8_t numDynMsgs = eepromStatus & 0x7F;

	DLOG(F("restoreBondData dynamic messages:"));
	DLOG(numDynMsgs, DEC);

  while(1) {

    len = EEPROM.read(addr);
    addr++;
    aciCmd.buffer[0] = len;

    DLOG(F("Restore message len:"));
    DLOG(len);

    for (uint8_t i = 1; i <= len; i++) {
        aciCmd.buffer[i] = EEPROM.read(addr);
        addr++;
    }
    waitForCmdComplete();
    DLOG(F("Send restore command"));
    if (!hal_aci_tl_send(&aciCmd)) {
      DLOG(F("restoreBondData: failed"));
      cmdComplete = true;
      return ACI_STATUS_ERROR_INTERNAL;
    }

    while (1) {
    	DLOG(F("Restore messages:"));
    	DLOG(numDynMsgs);
      if (lib_aci_event_get(&aciState, &aciData)) {
        DLOG(F("Event recieved:"));
        aciEvt = &aciData.evt;
        if (ACI_EVT_CMD_RSP != aciEvt->evt_opcode) {
            DLOG(F("restoreBondData: failed with error: 0x"));
            DLOG(aciEvt->evt_opcode, HEX);
            return ACI_STATUS_ERROR_INTERNAL;
        } else {
          numDynMsgs--;
          if (ACI_STATUS_TRANSACTION_COMPLETE == aciEvt->params.cmd_rsp.cmd_status) {
            bondedFirstTimeState = false;
            aciState.bonded = ACI_BOND_STATUS_SUCCESS;
    				DLOG(F("Restore of bond data completed successfully"));
            return ACI_STATUS_TRANSACTION_COMPLETE;
          }
          if (0 >= numDynMsgs) {
    				DLOG(F("Restore of bond data completed successfully"));
            return ACI_STATUS_ERROR_INTERNAL;
          }
          if (ACI_STATUS_TRANSACTION_CONTINUE == aciEvt->params.cmd_rsp.cmd_status) {
    				DLOG(F("Restore next bond data message"));
            break;
          }
        }
      }
    }
  }
}

bool BlueCapPeripheral::readAndWriteBondData() {
  bool status = false;
  aci_evt_t* aciEvt = NULL;
  uint8_t numDynMsgs = 0;
  uint8_t addr = eepromOffset + 1;

  lib_aci_read_dynamic_data();
  numDynMsgs++;

  DLOG(F("readAndWriteBondData"));

  while (1) {
  	DLOG(F("Message:"));
  	DLOG(numDynMsgs, DEC);
    if (true == lib_aci_event_get(&aciState, &aciData)) {
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

