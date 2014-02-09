#include <SPI.h>
#include "nordic/boards.h"
#include "nordic/lib_aci.h"
#include "nordic/aci_setup.h"
#include "dlog.h"

#include "blue_cap_peripheral.h"
#include "blue_cap_bond.h"

// public methods
BlueCapPeripheral::BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin) {
	init(_reqnPin, _rdynPin, NULL);
}

BlueCapPeripheral::BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin, BlueCapBond* _bond) {
	init(_reqnPin, _rdynPin, _bond);
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
void BlueCapPeripheral::init(uint8_t _reqnPin, uint8_t _rdynPin, BlueCapBond* _bond) {
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
					    uint8_t eepromStatus = bond->status();
              DLOG(F("found eepromStatus:"));
              DLOG(eepromStatus, HEX);
              if (eepromStatus != 0x00) {
                DLOG(F("Previous Bond present. Restoring"));
                if (ACI_STATUS_TRANSACTION_COMPLETE == bond->restoreBondData(eepromStatus, &aciState)) {
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
							if (bond->bondedFirstTimeState) {
								bond->bondedFirstTimeState = false;
								if (bond->readAndWriteBondData(&aciState)) {
									DLOG(F("Bond data read and store successful"));
								}
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

	if (bond) {
    bond->setup(&aciState);
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

