#ifndef _BLUE_CAP_PERIPHERAL_H
#define _BLUE_CAP_PERIPHERAL_H

#include "lib_aci.h"

#define BOND_HEADER_BYTES                 2

class BlueCapPeripheral {

public:

  BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin);
  BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin, uint16_t _eepromOffset);
  BlueCapPeripheral(uint8_t _reqnPin, uint8_t _rdynPin, uint16_t _eepromOffset, uint8_t _maxBonds);

  ~BlueCapPeripheral();

  virtual void begin(){setup();};
  virtual void loop(){listen();};

  void clearBondData();
  bool addBond();

  bool sendAck(const uint8_t pipe);
  bool sendNack(const uint8_t pipe, const uint8_t error_code);
  bool sendData(uint8_t pipe, uint8_t *value, uint8_t size);
  bool requestData(uint8_t pipe);
  bool setData(uint8_t pipe, uint8_t *value, uint8_t size);
  bool setTxPower(aci_device_output_power_t txPower);
  bool getBatteryLevel();
  bool getTemperature();
  bool getDeviceVersion();
  bool getBLEAddress();
  bool connect();
  bool bond();
  bool broadcast();
  bool radioReset();
  bool sleep();


protected:

  virtual void didReceiveData(uint8_t characteristicId, uint8_t* data, uint8_t size){};
  virtual void didReceiveCommandResponse(uint8_t commandId, uint8_t* data, uint8_t size){};
  virtual void didDisconnect(){};
  virtual void didTimeout(){};
  virtual void didConnect(){};
  virtual void didStartAdvertising(){};
  virtual void didReceiveError(uint8_t pipe, uint8_t errorCode){};
  virtual void didReceiveStatusChange(){};
  virtual void didBond(){};

  void setServicePipeTypeMapping(services_pipe_type_mapping_t* mapping, int count);
  void setSetUpMessages(hal_aci_data_t* messages, int count);

  bool isPipeAvailable(uint8_t pipe);
  virtual bool doTimingChange() = 0;

protected:

  bool                            broadcasting;

private:

  services_pipe_type_mapping_t*   servicesPipeTypeMapping;
  int                             numberOfPipes;
  hal_aci_data_t*                 setUpMessages;
  int                             numberOfSetupMessages;
  bool                            isConnected;
  bool                            ack;
  bool                            timingChangeDone;
  bool                            cmdComplete;
  uint8_t                         currentBondIndex;
  uint8_t*                        rxPipes;
  aci_state_t                     aciState;
  hal_aci_evt_t                   aciData;
  uint8_t                         reqnPin;
  uint8_t                         rdynPin;
  uint8_t                         maxBonds;

private:

  void init(uint8_t _reqnPin, uint8_t _rdynPin, uint16_t _eepromOffset, uint8_t _maxBonds, bool _broadcasting);
  void listen();
  void setup();
  void incrementCredit();
  void decrementCredit();
  void waitForCredit();
  void waitForAck();
  void waitForCmdComplete();
  uint8_t numberOfBondedDevices();
  uint8_t numberOfNewBonds();

private:

  class BlueCapBond {

    public:

      BlueCapBond();
      void init(BlueCapPeripheral* _peripheral, uint16_t _eepromOffset, uint16_t _maxBonds, uint8_t _index);
      void clearBondData();
      void setup(aci_state_t* aciState);
      bool restoreIfBonded(aci_state_t* aciState);
      void writeIfBonded(aci_state_t* aciState, aci_evt_t* aciEvt);
      void connectOrBond();

    public:

      uint16_t              eepromOffset;
      uint16_t              maxBonds;
      hal_aci_evt_t         aciData;
      bool                  bonded;
      uint8_t               index;
      bool                  newBond;
      BlueCapPeripheral*    peripheral;

    private:

      uint8_t status();
      aci_status_code_t restoreBondData(aci_state_t* aciState);
      bool readAndWriteBondData(aci_state_t* aciState);
      uint16_t writeBondData(aci_evt_t* evt, uint16_t addr);
      uint16_t readBondData(hal_aci_data_t* aciCmd, uint16_t addr);
      void writeBondDataHeader(uint16_t dataAddress, uint8_t numDynMsgs);
      uint16_t readBondDataOffset();
      uint16_t offset();
    };

private:

  BlueCapBond*              bonds;

private:

  void nextBondIndex();

};

class BlueCapBondedPeripheral : public BlueCapPeripheral {
public:
  BlueCapBondedPeripheral(uint8_t _reqnPin, uint8_t _rdynPin, uint16_t _eepromOffset, uint8_t _maxBonds) : BlueCapPeripheral(_reqnPin, _rdynPin, _eepromOffset, _maxBonds) {};
};

class BlueCapBroadcastingPeripheral : public BlueCapPeripheral {
public:
  BlueCapBroadcastingPeripheral(uint8_t _reqnPin, uint8_t _rdynPin) : BlueCapPeripheral(_reqnPin, _rdynPin){broadcasting = true;};
};

#endif

