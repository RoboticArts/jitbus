#ifndef SERIAL_JITBUS_H
#define SERIAL_JITBUS_H

#include <Arduino.h>
#include "core/jitcore.h"

// Remove auxiliar buffer for cobs, package will be built with enough spaces, add start and end frame. Keep length warning
// Check attempts
// Check virtual time ros
// Add bool inside the write function to know if data is sent. 

class SerialJitbus: public Jitcore { 
  
  public:

    SerialJitbus(){
       
        this->serial = &Serial;
        cbuffer = new CircularBuffer<uint8_t>(JITBUS_BUFFER_SIZE);

        #ifndef JITBUS_DISABLE_LOG
            this->serial_debug = &Serial;
            set_print_level(Jitlog::DEBUG);
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
 
        // Serial, for native USB
        for(int i = 0; i<bytes_to_write; i++){
            write(buffer[i]);
        }
 
    }

    uint32_t available_write(){

        return serial->availableForWrite();
    }


    bool write(uint8_t value){

        bool success = false;

        int bytes_written = serial->write(value);

        if (bytes_written <= 0){
            success = false;
        }
        else{
            success = true;
        }

        return success;

    }

    uint32_t available_read(){

        /*
        * If available_bytes is executed slower than the sender, the serial buffer
        * will overflow and the next received data will be lost. As a consequence,
        * the circular buffer will contain data packets corrupted and it will produce
        * wrong packets that will be discarted. This issue depends on the speed of
        * the device. If it can read fast, this problem will not appear. Choose a
        * data sending frequency according to hardware limitations. Avoid sleeps in
        * your code.
        * 
        * On serial ports is usually 64 bytes whileas on USB ports is always 64 
        * bytes. Unless you are sure that your device supports a larger buffer do
        * NOT change this value. 
        */

        if (serial->available() >= 63){ // 64 bytes buffer
            
            print_warn("SerialJitbus::available_bytes -> Serial RX buffer is full, "
                        "execution frequency is too low or sender is too fast. Next "
                        "bytes could be lost");
            
        }

        if (cbuffer->size() >= JITBUS_BUFFER_SIZE){ 
            
            print_warn("SerialJitbus::available_bytes -> Serial RX circular buffer is "
                       "full, execution frequency is toolow or sender is too fast. Next "
                       "bytes could be lost");
            
        }
        
        while(serial->available() > 0){

            cbuffer->push(serial->read());
        }

        uint32_t available_data = cbuffer->size(); 
        
        return available_data;
    }

    uint8_t read(){

        return cbuffer->pop();
    }


    virtual uint32_t time_ms(){

        return millis();
    }

    virtual uint32_t time_us(){

        return micros();
    }

  private:

    Stream* serial;
    CircularBuffer<uint8_t>* cbuffer;

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
