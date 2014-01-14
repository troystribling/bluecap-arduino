#include <SPI.h>

#include "blue_cap_peripheral.h"
#include "nordic/boards.h"
#include "nordic/lib_aci.h"
#include "nordic/aci_setup.h"
#include "dlog.h"

#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") ))

// public methods
BlueCapPeripheral::BlueCapPeripheral(uint8_t reqn, uint8_t rdyn) {
	init(reqn, rdyn, NULL, 0, NULL, 0);
}

BlueCapPeripheral::BlueCapPeripheral(uint8_t 					reqn,
														 				 uint8_t 				 	rdyn,
														 				 hal_aci_data_t* 	messages,
          			 										 int             	messagesCount) {
	init(reqn, rdyn, messages, messagesCount, NULL, 0);
}

BlueCapPeripheral::BlueCapPeripheral(uint8_t 											 reqn,
														 				 uint8_t 				 							 rdyn,
														 				 hal_aci_data_t*               messages,
          			 										 int                           messagesCount,
          			 										 services_pipe_type_mapping_t* mapping,
          			 										 int                           mappingCount) {
	init(reqn, rdyn, messages, messagesCount, mapping, mappingCount);
}

BlueCapPeripheral::~BlueCapPeripheral() {
}

void BlueCapPeripheral::begin() {
	aci_state.aci_setup_info.services_pipe_type_mapping = servicesPipeTypeMapping;
	aci_state.aci_setup_info.number_of_pipes    				= numberOfPipes;
	aci_state.aci_setup_info.setup_msgs         				= setUpMessages;
	aci_state.aci_setup_info.num_setup_msgs     				= numberOfSetupMessages;

		/*
	Tell the ACI library, the MCU to nRF8001 pin connections.
	The Active pin is optional and can be marked UNUSED
	*/
	aci_state.aci_pins.board_name = REDBEARLAB_SHIELD_V1_1;
	aci_state.aci_pins.reqn_pin   = reqn_pin;
	aci_state.aci_pins.rdyn_pin   = rdyn_pin;
	aci_state.aci_pins.mosi_pin   = MOSI;
	aci_state.aci_pins.miso_pin   = MISO;
	aci_state.aci_pins.sck_pin    = SCK;

#if defined(__SAM3X8E__)
	aci_state.aci_pins.spi_clock_divider     = 84;
#else
	aci_state.aci_pins.spi_clock_divider     = SPI_CLOCK_DIV8;
#endif

	aci_state.aci_pins.reset_pin             = UNUSED;
	aci_state.aci_pins.active_pin            = UNUSED;
	aci_state.aci_pins.optional_chip_sel_pin = UNUSED;

	aci_state.aci_pins.interface_is_interrupt	  = false;
	aci_state.aci_pins.interrupt_number			  = 1;

	//Turn debug printing on for the ACI Commands and Events to be printed on the Serial
	lib_aci_debug_print(true);

	/*We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
	and initialize the data structures required to setup the nRF8001*/
	lib_aci_init(&aci_state);
	delay(100);
}

void BlueCapPeripheral::listen() {
	if (lib_aci_event_get(&aci_state, &aci_data)) {
		aci_evt_t  *aci_evt;
		aci_evt = &aci_data.evt;
		switch(aci_evt->evt_opcode) {
			case ACI_EVT_DEVICE_STARTED:
				aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
				switch(aci_evt->params.device_started.device_mode) {
					case ACI_DEVICE_SETUP:
						DLOG(F("ACI_DEVICE_SETUP"));
						if (ACI_STATUS_TRANSACTION_COMPLETE != do_aci_setup(&aci_state)) {
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
				DLOG(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
				if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status) {
					DLOG(F("ACI_EVT_CMD_RSP: Error. Arduino is in an while(1); loop"));
					while (1);
				} else {
				}
				break;

			case ACI_EVT_CONNECTED:
				is_connected = 1;
				DLOG(F("ACI_EVT_CONNECTED"));
				aci_state.data_credit_available = aci_state.data_credit_total;
				didConnect();
				lib_aci_device_version();
				break;

			case ACI_EVT_PIPE_STATUS:
				DLOG(F("ACI_EVT_PIPE_STATUS"));
				// if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (timing_change_done == 0)) {
				// 	lib_aci_change_timing_GAP_PPCP();
				// 	timing_change_done = 1;
				// }
				break;

			case ACI_EVT_TIMING:
				DLOG(F("ACI_EVT_TIMING"));
				break;

			case ACI_EVT_DISCONNECTED:
				is_connected = 0;
				ack = 1;
				DLOG(F("ACI_EVT_DISCONNECTED"));
				didDisconnect();
				lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
				DLOG(F("Advertising started"));
				didStartAdvertising();
				break;

			case ACI_EVT_DATA_RECEIVED: {
				int pipe = aci_evt->params.data_received.rx_data.pipe_number;
				int length = aci_evt->len - 2;
				DLOG(F("ACI_EVT_DATA_RECEIVED Pipe #:"));
				DLOG(pipe, HEX);
				DLOG(F("length:"));
				DLOG(length, DEC);
				didReceiveData(pipe, aci_evt->params.data_received.rx_data.aci_data, length);
				break;
			}

			case ACI_EVT_DATA_CREDIT:
				aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
				DLOG(F("ACI_EVT_DATA_CREDIT"));
				DLOG(aci_state.data_credit_available,DEC);
				ack=1;
				break;

			case ACI_EVT_PIPE_ERROR:
				//See the appendix in the nRF8001 Product Specication for details on the error codes
				DLOG(F("ACI_EVT_PIPE_ERROR: Pipe #:"));
				DLOG(aci_evt->params.pipe_error.pipe_number, DEC);
				DLOG(F("Pipe Error Code: 0x"));
				DLOG(aci_evt->params.pipe_error.error_code, HEX);

				//Increment the credit available as the data packet was not sent
				aci_state.data_credit_available++;
				DLOG(F("Data Credit available:"));
				DLOG(aci_state.data_credit_available,DEC);
				break;
		}
	}
}

unsigned char BlueCapPeripheral::connected() {
    return is_connected;
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
void BlueCapPeripheral::init(uint8_t 											 reqn,
														 uint8_t 											 rdyn,
														 hal_aci_data_t*               messages,
          				 					 int                           messagesCount,
          				 					 services_pipe_type_mapping_t* mapping,
          				 					 int                           mappingCount) {

	setUpMessages = messages;
	numberOfSetupMessages = messagesCount;
	servicesPipeTypeMapping = mapping;
	numberOfPipes = mappingCount;
	is_connected = 0;
	ack = 0;
	reqn_pin = reqn;
	rdyn_pin = rdyn;
	timing_change_done = 0;
}

void BlueCapPeripheral::writeBuffers() {
	// if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX)) {
	// 	if(tx_buffer_len > 0) {
	// 		unsigned char Index = 0;
	// 		while(tx_buffer_len > 20) {
	// 			if(true == lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, &tx_buff[Index], 20)) {
	// 				DLOG(F("data transmmit success!  Length: "));
	// 				DLOG(20, DEC);
	// 			}
	// 			else {
	// 				DLOG("data transmmit fail !");
	// 			}
	// 			tx_buffer_len -= 20;
	// 			Index += 20;
	// 			aci_state.data_credit_available--;
	// 			DLOG(F("Data Credit available: "));
	// 			DLOG(aci_state.data_credit_available,DEC);
	// 			ack = 0;
	// 			while (!ack)
	// 				processEvents();
	// 		}

	// 		if(true == lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX,& tx_buff[Index], tx_buffer_len)) {
	// 			DLOG(F("data transmmit success!  Length: "));
	// 			DLOG(tx_buffer_len, DEC);
	// 		}
	// 		else {
	// 			DLOG(F("data transmmit fail !"));
	// 		}
	// 		tx_buffer_len = 0;
	// 		aci_state.data_credit_available--;
	// 		DLOG(F("Data Credit available: "));
	// 		DLOG(aci_state.data_credit_available,DEC);
	// 		ack = 0;
	// 		while (!ack)
	// 			processEvents();
	// 	}
	// }
	// processEvents();
}


