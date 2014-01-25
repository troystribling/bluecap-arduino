#ifndef _BLUE_CAP_PERIPHERAL_H
#define _BLUE_CAP_PERIPHERAL_H

#include "nordic/lib_aci.h"

class BlueCapPeripheral {

public:

  BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin);
  BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin, bool _bond);

  ~BlueCapPeripheral();

  virtual void begin(){setup();};
  virtual void loop(){listen();};

  bool connected(void);
  bool sendAck(const uint8_t pipe);
  bool sendNack(const uint8_t pipe, const uint8_t error_code);
  bool sendData(uint8_t pipe, uint8_t *value, uint8_t size);
  bool requestData(uint8_t pipe);
  bool setData(uint8_t pipe, uint8_t *value, uint8_t size);
  bool getBatteryLevel();
  bool getTemperature();
  bool getDeviceVersion();
  bool getAddress();

protected:

  virtual void didReceiveData(uint8_t characteristicId, uint8_t* data, uint8_t size){};
  virtual void didReceiveCommandResponse(uint8_t commandId, uint8_t* data, uint8_t size){};
  virtual void didDisconnect(){};
  virtual void didConnect(){};
  virtual void didStartAdvertising(){};
  virtual void didReceiveError(uint8_t pipe, uint8_t errorCode){};
  virtual void didReceiveStatusChange(){};

  void setServicePipeTypeMapping(services_pipe_type_mapping_t* mapping, int count);
  void setSetUpMessages(hal_aci_data_t* messages, int count);

  bool isPipeAvailable(uint8_t pipe);
  virtual bool doTimingChange() = 0;

private:

  services_pipe_type_mapping_t*   servicesPipeTypeMapping;
  int                             numberOfPipes;
  hal_aci_data_t*                 setUpMessages;
  int                             numberOfSetupMessages;
  bool                            isConnected;
  bool                            ack;
  bool                            timingChangeDone;
  bool                            bond;
  bool                            cmdComplete;
  uint8_t                         reqnPin;
  uint8_t                         rdynPin;
  uint8_t*                        rxPipes;
  aci_state_t                     aciState;
  hal_aci_evt_t                   aciData;

private:

  void init(uint8_t _reqnPin, uint8_t _rdynPin, bool _bond);

  void listen();
  void setup();
  void incrementCredit();
  void decrementCredit();
  void waitForCredit();
  void waitForAck();
  void waitForEEPROM();
  void waitForCmdComplete();

};

#endif

