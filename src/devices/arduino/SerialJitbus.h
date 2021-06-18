#ifndef SERIAL_JITBUS_H
#define SERIAL_JITBUS_H

#include <Arduino.h>
#include "core/jitcore.h"

class SerialJitbus: public Jitcore { 
  
  public:

    SerialJitbus(){
       
        this->serial = &Serial;

        #ifndef JITBUS_DISABLE_LOG
            this->serial_debug = &Serial;
        #endif
    
    }

    ~SerialJitbus(){

    }


    void begin(Stream &serial){

        this->serial = &serial;

    } 

    void begin(Stream &serial, Stream &serial_debug){

        this->serial = &serial;
        
        #ifndef JITBUS_DISABLE_LOG
            enable_color = false;
            this->serial_debug = &serial_debug; 
        #endif
    } 


    void waitSerialUSB(void){
        
        while(!Serial){;}
    }


    void write(uint8_t* buffer, int bytes_to_write){
    
        for(int i = 0; i<bytes_to_write; i++){ 
            serial->write(buffer[i]);
        }

    }


    void read(CircularBuffer<uint8_t>* cbuffer, int bytes_to_read){
        

        for (int i = 0; i<bytes_to_read; i++){

            if (serial->available() > 0){
                
                cbuffer->push(serial->read());
                
            }
            else{
                break;
            }
        } 

       /*
        uint32_t pending_bytes = serial->available();

        for (uint32_t i = 0; i<pending_bytes; i++){

            cbuffer->push(serial->read());
        }
    */
    }


  private:

    Stream* serial;

    #ifndef JITBUS_DISABLE_LOG

        Stream* serial_debug;

        void print_log(const char* message){

            serial_debug->print(message);
        }

    #else

        void print_log(const char* message){}

    #endif

};



#endif
