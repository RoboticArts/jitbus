#ifndef JITCORE_H
#define JITCORE_H

#include "cbuffer.h"
#include "jitpacket.h"
#include "jitlog.h"

#ifndef JITBUS_BUFFER_SIZE
  #define JITBUS_BUFFER_SIZE 100
#endif

#ifndef JITBUS_MIN_BUFFER_SPACE
  #define JITBUS_MIN_BUFFER_SPACE 10
#endif

class Jitcore: public Jitlog{ 
  
  public:

    Jitcore(){
      cbuffer = new CircularBuffer<uint8_t>(JITBUS_BUFFER_SIZE);
      // _fake_read(cbuffer);

      packet_available = false;
      packet_length = 0;
      packet_id = 0;

    };

    ~Jitcore(){

    };

    virtual void write(uint8_t* data, int bytes_to_write){

    }

    virtual void read(CircularBuffer<uint8_t>* cbuffer, int bytes_to_read){

    }

    bool available(){
       
        read(cbuffer, cbuffer->free_size());
        //read(cbuffer, 200);

        if (cbuffer->size() > JITBUS_MIN_BUFFER_SPACE){

          JitPacket jitpacket;

          if (jitpacket.findPacket(cbuffer) == true){

            packet_available = true;
            packet_length = jitpacket.packetLength();
            packet_id = jitpacket.packetId();
          }

          else{

            packet_available = false;
            packet_length = 0;
            packet_id = 0;

          }
        }

        else{
          packet_available = false;
        }

        return packet_available;
    }

    template<typename Type>
    bool receivePacket(const Type& data, uint16_t data_id){

      bool is_received = false;

        if (packet_available == true){

          if (data_id == packet_id){

            JitPacket jitpacket;
            jitpacket.parsePacket(data, cbuffer, packet_length);
            packet_available = false;
            is_received = true;
          }

        }

      return is_received;
    }

    template<typename Type>
    void sendPacket(const Type& data, uint16_t data_id){
        
        JitPacket jitpacket;
        jitpacket.buildPacket(data, data_id);

        write(jitpacket.buffer, jitpacket.packetLength());

    }



  private:

    CircularBuffer<uint8_t>* cbuffer;
    bool packet_available;
    int packet_length;
    int packet_id;

    void _fake_read(CircularBuffer<uint8_t>* cbuffer){

      packet_available = true;
      packet_id = 66;
      
      //garbage
      //cbuffer->push(215);
      //cbuffer->push(176);
      //cbuffer->push(51);
      //start frame
      cbuffer->push(255);
      cbuffer->push(255);
      //total length
      cbuffer->push(0);
      cbuffer->push(0);
      cbuffer->push(0);
      cbuffer->push(18);
      //id
      cbuffer->push(0);
      cbuffer->push(66);
      //data
      cbuffer->push(128);
      cbuffer->push(62);
      cbuffer->push(17);
      cbuffer->push(0);
      cbuffer->push(87);
      cbuffer->push(0);
      //crc
      cbuffer->push(0);
      cbuffer->push(0);
      //end frame
      cbuffer->push(254);
      cbuffer->push(127);
      //garbage
      //cbuffer->push(111);
      //cbuffer->push(222);

    }

};



#endif
