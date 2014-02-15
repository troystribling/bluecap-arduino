#include <SPI.h>
#include "nordic/boards.h"
#include "nordic/lib_aci.h"
#include "nordic/aci_setup.h"
#include "dlog.h"

#include "blue_cap_peripheral.h"

#define REMOTE_COMMAND(X, Y, Z) bool BlueCapPeripheral::X {             \
  bool status = false;                                                  \
  DLOG(F(Z));                                                           \
  if (isPipeAvailable(pipe)) {                                          \
    waitForCredit();                                                    \
    status = Y;                                                         \
  }                                                                     \
  if (status) {                                                         \
    waitForAck();                                                       \
    DLOG(F("successful over pipe:"));                                   \
  } else {                                                              \
    DLOG(F("failed over pipe:"));                                       \
 }                                                                      \
  DLOG(pipe, HEX);                                                      \
  return status;                                                        \
}                                                                       \

#define LOCAL_COMMAND(X, Y, Z) bool BlueCapPeripheral::X {              \
  waitForCmdComplete();                                                 \
  DLOG(F(Z));                                                           \
  bool status = Y;                                                      \
  if (status) {                                                         \
    DLOG(F("successful"));                                              \
  } else {                                                              \
    DLOG(F("failed"));                                                  \
    cmdComplete = true;                                                 \
  }                                                                     \
  return status;                                                        \
}                                                                       \

// public methods
BlueCapPeripheral::BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin) {
	init(_reqnPin, _rdynPin, 0, 0);
}

BlueCapPeripheral::BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin, uint16_t _eepromOffset) {
  init(_reqnPin, _rdynPin, _eepromOffset, 0);
}

BlueCapPeripheral::BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin, uint16_t _eepromOffset, uint8_t _maxBonds) {
  init(_reqnPin, _rdynPin, _eepromOffset, _maxBonds);
}

BlueCapPeripheral::~BlueCapPeripheral() {
  for(int i = 0; i < maxBonds; i++) {
    delete bonds[i];
  }
}

void  BlueCapPeripheral::clearBondData() {
  for(int i = 0; i < maxBonds; i++) {
    bonds[i]->clearBondData();
  }
}

REMOTE_COMMAND(sendAck(uint8_t pipe), lib_aci_send_ack(&aciState, pipe), "sendAck")
REMOTE_COMMAND(sendNack(uint8_t pipe, const uint8_t errorCode), lib_aci_send_nack(&aciState, pipe, errorCode), "sendNack")
REMOTE_COMMAND(sendData(uint8_t pipe, uint8_t* value, uint8_t size), lib_aci_send_data(pipe, value, size), "sendData")
REMOTE_COMMAND(requestData(uint8_t pipe), lib_aci_request_data(&aciState, pipe), "requestData")

LOCAL_COMMAND(setData(uint8_t pipe, uint8_t* value, uint8_t size), lib_aci_set_local_data(&aciState, pipe, value, size), "setData")
LOCAL_COMMAND(setTxPower(aci_device_output_power_t txPower), lib_aci_set_tx_power(txPower), "setTxPower")
LOCAL_COMMAND(getBatteryLevel(), lib_aci_get_battery_level(), "getBatteryLevel")
LOCAL_COMMAND(getTemperature(), lib_aci_get_temperature(), "getTemperartue")
LOCAL_COMMAND(getDeviceVersion(), lib_aci_device_version(), "getDeviceVersion")
LOCAL_COMMAND(getAddress(), lib_aci_get_address(), "getAddress")

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
void BlueCapPeripheral::init(uint8_t _reqnPin, uint8_t _rdynPin, uint16_t _eepromOffset, uint8_t _maxBonds) {
	setUpMessages = NULL;
	numberOfSetupMessages = 0;
	servicesPipeTypeMapping = NULL;
	numberOfPipes = 0;
	isConnected = false;
	ack = false;
	timingChangeDone = false;
	cmdComplete = true;
  currentBondIndex = 0;
  reqnPin = _reqnPin;
  rdynPin = _rdynPin;
  maxBonds = _maxBonds;
  if (maxBonds > 0) {
    bonds = (BlueCapBond**)malloc(maxBonds*sizeof(BlueCapBond*));
    for (int i = 0; i < maxBonds; i++) {
      bonds[i] = new BlueCapBond(_eepromOffset, maxBonds, i);
    }
  } else {
    bonds = NULL;
  }
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
            if (maxBonds > 0) {
              if (currentBond()->deviceStandByReceived(&aciState)) {
                didStartAdvertising();
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
        if (maxBonds > 0) {
          currentBond()->disconnected(&aciState, aciEvt);
        } else {
  				lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
  				DLOG(F("Advertising started"));
        }
        didStartAdvertising();
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

  for(int i = 0; i < maxBonds; i++) {
    bonds[i]->setup(&aciState);
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
	DLOG(aciState.data_credit_available, DEC);
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

// BlueCapBond
BlueCapPeripheral::BlueCapBond* BlueCapPeripheral::currentBond() {
  BlueCapBond* bond = NULL;
  if (currentBondIndex < maxBonds){
    bond = bonds[currentBondIndex];
  }
  return bond;
}

void BlueCapPeripheral::nextBond() {
  currentBondIndex++;
  if (currentBondIndex > maxBonds - 1) {
    currentBondIndex = 0;
  }
}
