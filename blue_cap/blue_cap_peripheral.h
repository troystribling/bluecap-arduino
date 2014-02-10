#ifndef _BLUE_CAP_PERIPHERAL_H
#define _BLUE_CAP_PERIPHERAL_H

#include "nordic/lib_aci.h"

class BlueCapPeripheral {

public:

  BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin);
  BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin, uint16_t _eepromOffset);

  ~BlueCapPeripheral();

  virtual void begin(){setup();};
  virtual void loop(){listen();};

  bool sendAck(const uint8_t pipe);
  bool sendNack(const uint8_t pipe, const uint8_t error_code);
  bool sendData(uint8_t pipe, uint8_t *value, uint8_t size);
  bool requestData(uint8_t pipe);
  bool setData(uint8_t pipe, uint8_t *value, uint8_t size);
  bool setTxPower(aci_device_output_power_t txPower);
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
  virtual void didBond(){};

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
  bool                            cmdComplete;
  uint8_t                         reqnPin;
  uint8_t                         rdynPin;
  uint8_t*                        rxPipes;
  aci_state_t                     aciState;
  hal_aci_data_t                  aciCmd;
  hal_aci_evt_t                   aciData;

private:

  void init(uint8_t _reqnPin, uint8_t _rdynPin);

  void listen();
  void setup();
  void incrementCredit();
  void decrementCredit();
  void waitForCredit();
  void waitForAck();
  void waitForCmdComplete();

private:

  class BlueCapBond {

    public:

      BlueCapBond(uint16_t _eepromOffset);

      void clearBondData();
      uint8_t status();
      void setup(aci_state_t* aciState);

      uint16_t writeBondData(aci_evt_t* evt, uint16_t addr);
      aci_status_code_t restoreBondData(uint8_t eepromStatus, aci_state_t* aciState);
      bool readAndWriteBondData(aci_state_t* aciState);
      bool deviceStandByReceived(aci_state_t* aciState);
      bool disconnected(aci_state_t* aciState);

    private:

      uint16_t              eepromOffset;
      hal_aci_data_t        aciCmd;
      hal_aci_evt_t         aciData;
      bool                  bondedFirstTimeState;

    private:

      void init(uint16_t _eepromOffset);

    };

    BlueCapBond*            bond;

public:

    void clearBondData();

};

#endif

