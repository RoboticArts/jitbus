#ifndef SERIAL_JITBUS_H
#define SERIAL_JITBUS_H

#define JITLOG_SIZE 1000

#include <chrono>
#include <thread>
#include <time.h>
#include <iostream> 
#include <libserial/SerialPort.h>
#include "core/jitcore.h"

// TODO(robert): Minimal documentation
// TODO(robert): Check reconnection

class SerialJitbus: public Jitcore {

  public:
	
    SerialJitbus()
    {
      cbuffer = new CircularBuffer<uint8_t>(JITBUS_BUFFER_SIZE);
      enable_color = true;
      time(&last_time);
      time(&current_time);
      last_state = CONNECTED;
      current_state = DISCONNECTED;
      set_print_level(Jitlog::INFO);
    }

    ~SerialJitbus()
    {
		  disconnect();
    }

    bool init(const char* serial_port, int baudrate, int timeout = 0, int reconnection_time = 2)
    {
      serial_port_ = serial_port;
      baudrate_ = baudrate;
      timeout_ = timeout;
      reconnection_time_ = reconnection_time; 
		  
      return connect();
	  }

    bool connect()
    {
      try
      {
        serial = std::make_shared<LibSerial::SerialPort>();

        serial->Open(serial_port_);
        serial->SetBaudRate(getBaudRateEnum(baudrate_));

        print_info("jitbus::connect -> Serial port %s opened", serial_port_);

        switchState(CONNECTED);
        return true;
		  }
      catch(const LibSerial::OpenFailed& e)
      {
        print_error("jitbus::connect -> Unable to open port: %s", serial_port_);
        switchState(DISCONNECTED);
		  }
      catch (const std::exception& e)
      {
        print_error("jitbus::connect -> std::exception: %s", e.what());
        switchState(DISCONNECTED);
      }

      return false;
    }

    void disconnect()
    {
      serial->Close();
      switchState(DISCONNECTED);
    }

    bool connected()
    {
      bool is_connected = false;

      if (current_state == CONNECTED)
      {
        is_connected = true;
        time(&last_time);	
      }
      else
      {
        time(&current_time);
        
        if (difftime(current_time, last_time) >= reconnection_time_)
        {
          is_connected = connect();
          time(&last_time);
        }
      }

      return is_connected;
    }

    void waitForSerial(void)
    {

    }

    bool write(uint8_t data)
    {
      if (connected())
      {
        try
        {
          serial->WriteByte(data);

          // If no exception is thrown, we assume the write was accepted
          // by the OS (kernel buffer)
          return true;
        }
        catch (const LibSerial::NotOpen& e)
        {
          print_error("jitbus::write -> Port not open: %s", serial_port_);
          switchState(DISCONNECTED);
        }
        catch (const std::exception& e)
        {
          print_error("jitbus::write -> std::exception: %s", e.what());
          switchState(DISCONNECTED);
        }
		  }

		  return false; 
    }

    uint32_t available_write()
    {
      uint32_t virtual_serial_size = 64;
      return virtual_serial_size;
    }

    uint32_t available_read(){

      if (connected())
      {
        if (serial->GetNumberOfBytesAvailable() >= 63) // 64 bytes buffer
        {          
          print_warn("SerialJitbus::available_bytes -> Serial RX buffer is full, "
                "execution frequency is too low or sender is too fast. Next "
                "bytes could be lost");  
        }

        if (cbuffer->size() >= JITBUS_BUFFER_SIZE)
        {
          print_warn("SerialJitbus::available_bytes -> Serial RX circular buffer is "
              "full, execution frequency is too low or sender is too fast. Next "
              "bytes could be lost");   
        }
        
        while(serial->GetNumberOfBytesAvailable() > 0)
        {
          if (connected())
          {
            try
            {
              uint8_t read_byte;
              serial->ReadByte(read_byte, timeout_);
              cbuffer->push(read_byte);
            }
            catch(const LibSerial::NotOpen& e)
            {
                print_error("jitbus::read -> Port not open: %s", e.what());
                switchState(DISCONNECTED);
            }
            catch(const std::exception& e)
            {
                print_error("jitbus::read -> std::exception: %s", e.what());
                switchState(DISCONNECTED);
            }
          }
        }
        
      }

      uint32_t available_data = cbuffer->size(); 
      return available_data;
    }

    uint8_t read()
    {
      return cbuffer->pop();
    }

    void print_log(const char* message)
    {
      printf("%s", message);
    }

    uint32_t time_ms()
    {
      std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
      uint32_t time_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end-time_begin).count();

      return time_now_ms;
    }


    uint32_t time_us()
    {
      std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
      uint32_t time_now_us = std::chrono::duration_cast<std::chrono::microseconds>(time_end-time_begin).count();

      return time_now_us;
    }

	void delay_ms(uint32_t wait_ms)
  {
		std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
	}

	void delay_us(uint32_t wait_us)
  {
		std::this_thread::sleep_for(std::chrono::microseconds(wait_us));
	}

  LibSerial::BaudRate getBaudRateEnum(int baudrate) 
  {
    switch (baudrate) 
    {
        case 110: return LibSerial::BaudRate::BAUD_110;
        case 300: return LibSerial::BaudRate::BAUD_300;
        case 600: return LibSerial::BaudRate::BAUD_600;
        case 1200: return LibSerial::BaudRate::BAUD_1200;
        case 2400: return LibSerial::BaudRate::BAUD_2400;
        case 4800: return LibSerial::BaudRate::BAUD_4800;
        case 9600: return LibSerial::BaudRate::BAUD_9600;
        case 19200: return LibSerial::BaudRate::BAUD_19200;
        case 38400: return LibSerial::BaudRate::BAUD_38400;
        case 57600: return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        case 230400: return LibSerial::BaudRate::BAUD_230400;
        case 460800: return LibSerial::BaudRate::BAUD_460800;
        case 921600: return LibSerial::BaudRate::BAUD_921600;
        default:
            std::cerr << "Baudrate " << baudrate << " is not supported. Exit program." << std::endl;
            std::exit(EXIT_FAILURE);
    }
  }

  private:

    CircularBuffer<uint8_t>* cbuffer;

    enum state_enum {CONNECTED, DISCONNECTED}; 
    int current_state;
    int last_state;

    std::shared_ptr<LibSerial::SerialPort> serial;

    const char* serial_port_;
    int baudrate_;
    int timeout_;
    int reconnection_time_;

    time_t current_time, last_time;

    std::chrono::steady_clock::time_point time_begin = std::chrono::steady_clock::now();

    void switchState(int state)
    {
      if (state != current_state)
      {
        last_state = current_state;
        current_state = state;
      }
    }

};

#endif