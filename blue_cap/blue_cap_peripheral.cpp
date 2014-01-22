#include <SPI.h>

#include "blue_cap_peripheral.h"
#include "nordic/boards.h"
#include "nordic/lib_aci.h"
#include "nordic/aci_setup.h"
#include "dlog.h"

// public methods
BlueCapPeripheral::BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin) {
	init(_reqnPin, _rdynPin, false);
}

BlueCapPeripheral::BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin, bool _bond) {
	init(_reqnPin, _rdynPin, _bond);
}

BlueCapPeripheral::~BlueCapPeripheral() {
}

bool BlueCapPeripheral::connected() {
    return isConnected;
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

bool BlueCapPeripheral::setData(uint8_t pipe, uint8_t *value, uint8_t size) {
	bool status = false;
	// if (isPipeAvailable(pipe)) {
		waitForCredit();
		status = lib_aci_set_local_data(&aciState, pipe, value, size);
	// }
	if (status) {
		waitForAck();
		DLOG(F("setData successful over pipe:"));
	} else {
		DLOG(F("sendData failed over pipe:"));
	}
	DLOG(pipe, HEX);
	return status;
}

bool BlueCapPeripheral::getBatteryLevel() {
	return lib_aci_get_battery_level();
}


void BlueCapPeripheral::setServicePipeTypeMapping(services_pipe_type_mapping_t* mapping, int count) {
	servicesPipeTypeMapping = mapping;
	numberOfPipes = count;
}

void BlueCapPeripheral::setSetUpMessages(hal_aci_data_t* messages, int count) {
	setUpMessages = messages;
	numberOfSetupMessages = count;
}

// private methods
void BlueCapPeripheral::init(uint8_t _reqnPin, uint8_t _rdynPin, bool _bond) {

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
					case ACI_DEVICE_STANDBY:
						DLOG(F("ACI_DEVICE_STANDBY"));
						lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
						didStartAdvertising();
						DLOG(F("Advertising started"));
						break;
				}
				break;

			case ACI_EVT_CMD_RSP:
				DLOG(F("ACI_EVT_CMD_RSP"));
				DLOG(aciEvt->params.cmd_rsp.cmd_opcode, HEX);
				if (ACI_STATUS_SUCCESS != aciEvt->params.cmd_rsp.cmd_status) {
					DLOG(F("ACI_EVT_CMD_RSP: Error. Arduino is in an while(1); loop"));
					while (1);
				} else {
				}
				break;

			case ACI_EVT_CONNECTED:
				isConnected = true;
				timingChangeDone = false;
				DLOG(F("ACI_EVT_CONNECTED"));
				aciState.data_credit_available = aciState.data_credit_total;
				didConnect();
				lib_aci_device_version();
				break;

      case ACI_EVT_BOND_STATUS:
        aciState.bonded = aciEvt->params.bond_status.status_code;
        break;

			case ACI_EVT_PIPE_STATUS:
				DLOG(F("ACI_EVT_PIPE_STATUS"));
				if (arePipesAvailable() && (timingChangeDone == false)) {
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
				lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
				DLOG(F("Advertising started"));
				didStartAdvertising();
				break;

			case ACI_EVT_DATA_RECEIVED: {
				int pipe = aciEvt->params.data_received.rx_data.pipe_number;
				int length = aciEvt->len - 2;
				DLOG(F("ACI_EVT_DATA_RECEIVED Pipe #:"));
				DLOG(pipe, HEX);
				DLOG(F("length:"));
				DLOG(length, DEC);
				didReceiveData(pipe, aciEvt->params.data_received.rx_data.aci_data, length);
				break;
			}

			case ACI_EVT_DATA_CREDIT:
				aciState.data_credit_available = aciState.data_credit_available + aciEvt->params.data_credit.credit;
				DLOG(F("ACI_EVT_DATA_CREDIT"));
				DLOG(aciState.data_credit_available, DEC);
				ack = true;
				break;

			case ACI_EVT_PIPE_ERROR:
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

	//Turn debug printing on for the ACI Commands and Events to be printed on the Serial
	lib_aci_debug_print(true);

	/*We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
	and initialize the data structures required to setup the nRF8001*/
	lib_aci_init(&aciState);
	delay(100);
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
