#ifndef JITCORE_H
#define JITCORE_H

#ifndef JITBUS_BUFFER_SIZE
  #define JITBUS_BUFFER_SIZE 1000
#endif

#include "cbuffer.h"
#include "cobs.h"
#include "crc.h"
#include "jitlog.h"
#include "jitinterface.h"
#include "jitpacket.h"
#include "jitstream.h"


class Jitcore: public JitStream{ 
  
  public:

    Jitcore(){};
    ~Jitcore(){

    };

    bool available(){

        return JitStream::availablePacket();
    }

    template<typename Type>
    bool receivePacket(const Type& data, uint16_t data_id){
   
        return JitStream::readPacket(data, data_id);
    }

    template<typename Type>
    void sendPacket(const Type& data, uint16_t data_id){

        JitStream::writePacket(data, data_id);

    }

    template<typename Type>
    bool sendPacketHz(const Type& data, uint16_t data_id, uint32_t& time, float frequency){

        return JitStream::writePacketHz(data, data_id, time, frequency);
    }

    template<typename Type>
    bool sendPacketBlocking(const Type& data, uint16_t data_id, uint32_t timeout = 0){

        return JitStream::writePacketBlocking(data, data_id, timeout);
    }


  private:


};



#endif
