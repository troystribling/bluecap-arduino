#ifndef _BLUE_CAP_PERIPHERAL_H
#define _BLUE_CAP_PERIPHERAL_H

#include "nordic/lib_aci.h"

class BlueCapPeripheral {

public:

  BlueCapPeripheral();
  BlueCapPeripheral(hal_aci_data_t* messages,
                    int             messagesCount);
  BlueCapPeripheral(hal_aci_data_t*               messages,
                    int                           messagesCount,
                    services_pipe_type_mapping_t* mapping,
                    int                           mappingCount);
  ~BlueCapPeripheral();

  void begin();
  void listen();

  int read();
  void write(unsigned char data);
  void writeBytes(unsigned char *data, unsigned char len);

  unsigned char available();
  unsigned char connected(void);

  void setServicePipeTypeMapping(services_pipe_type_mapping_t* mapping, int count);
  void setSetUpMessages(hal_aci_data_t* messages, int count);

protected:

private:

  services_pipe_type_mapping_t*   servicesPipeTypeMapping;
  int                             numberOfPipes;
  hal_aci_data_t*                 setUpMessages;
  int                             numberOfSetupMessages;

private:

  void init(hal_aci_data_t*               messages,
            int                           messagesCount,
            services_pipe_type_mapping_t* mapping,
            int                           mappingCount);

  void writeBuffers();
};

#endif

