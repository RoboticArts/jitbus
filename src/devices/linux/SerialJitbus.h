#ifndef SERIAL_JITBUS_H
#define SERIAL_JITBUS_H

#define JITLOG_SIZE 1000

#include <chrono>
#include <thread>
#include <time.h>
#include "serial/serial.h"
#include "core/jitcore.h"

class SerialJitbus: public Jitcore { 

  public:
	
    SerialJitbus(){

		cbuffer = new CircularBuffer<uint8_t>(JITBUS_BUFFER_SIZE);
		enable_color = true;
		time(&last_time);
		time(&current_time);
		last_state = CONNECTED;
		current_state = DISCONNECTED;
		set_print_level(Jitlog::INFO);
    }

    ~SerialJitbus(){

		disconnect();
    }

    void init(const char* serial_port, int baudrate, int timeout = 50, int reconnection_time = 2){
			
		this->serial_port = serial_port;
		this->baudrate = baudrate;
		this->timeout = timeout;
		this->reconnection_time = reconnection_time; 

		connect();
		
	}



	bool connect(){

		bool success;

		try{

			serial.setPort(serial_port);
			serial.setBaudrate(baudrate);

			serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
			serial.setTimeout(to);

			serial.open();
			print_info("jitbus::connect -> Serial port %s opened", serial_port);

			switchState(CONNECTED);
			success = true;
		}

		catch(serial::IOException& e){
			
			print_error("ERROR: jitbus::connect -> Unable to open port %s", serial_port);
			switchState(DISCONNECTED);
			success = false;
		}


		return success;
	}

	void disconnect(){

		serial.close();
		switchState(DISCONNECTED);
	}
/*
	bool reconnect(){

		bool success;

		try{
			serial.open();
			status = CONNECTED;
			success = true;
			print_info("jitbus::reconnect -> Serial port %s reconnected", serial_port);
		}
		catch(serial::IOException& e){
			status = DISCONNECTED;
			success = false;
			print_error("jitbus::reconnect -> Unable to reconnect serial port, trying again in 5 seconds...");
		}

		return success;
	}
*/
	bool connected(){

		bool is_connected;

		if (current_state == CONNECTED){

			is_connected = true;
			time(&last_time);	
		}
		else{

			time(&current_time);
			
			if (difftime(current_time, last_time) >= reconnection_time){

				is_connected = connect();
				time(&last_time);
			}
		}

		return is_connected;
	}

    void waitForSerial(void){

    }

    bool write(uint8_t data){

		bool success = false;

		if (connected()){

			try{

				size_t bytes_written = serial.write(&data,1);
				
				if (bytes_written <= 0){
					success = false;
					print_warn("jitbus::write -> Byte hex: %X not sent, I will try again", data);
				}
				else{
					success = true;
				}
			}

			catch(serial::IOException& e){
		
				print_error("jitbus::write -> %s",e.what());
				switchState(DISCONNECTED);
			}
			catch(serial::PortNotOpenedException& e){
				print_error("jitbus::write -> %s",e.what());
				switchState(DISCONNECTED);
			}
			catch (serial::SerialException& e){
				print_error("jitbus::write -> %s",e.what());
				switchState(DISCONNECTED);
			}

		}

		return success; 

    }

    uint32_t available_write(){
		
		uint32_t virtual_serial_size = 64;

        return virtual_serial_size;
    }

	uint32_t available_read(){

		if (connected()){

			if (serial.available() >= 63){ // 64 bytes buffer
				
				print_warn("SerialJitbus::available_bytes -> Serial RX buffer is full, "
							"execution frequency is too low or sender is too fast. Next "
							"bytes could be lost");
				
			}

			if (cbuffer->size() >= JITBUS_BUFFER_SIZE){ 
				
				print_warn("SerialJitbus::available_bytes -> Serial RX circular buffer is "
						"full, execution frequency is too low or sender is too fast. Next "
						"bytes could be lost");
				
			}
			
			while(serial.available() > 0){

				if (connected()){

					try{

						cbuffer->push(serial.read().c_str()[0]);
						
					}

					catch(serial::IOException& e){
				
						print_error("jitbus::read -> %s",e.what());
						switchState(DISCONNECTED);
					}
					catch(serial::PortNotOpenedException& e){
						print_error("jitbus::read -> %s",e.what());
						switchState(DISCONNECTED);
					}
					catch (serial::SerialException& e){
						print_error("jitbus::read -> %s",e.what());
						switchState(DISCONNECTED);
					}

				}
			}
		}

        uint32_t available_data = cbuffer->size(); 
        
        return available_data;

	}


	uint8_t read(){

        return cbuffer->pop();
    }


	void print_log(const char* message){

		printf("%s", message);

	}

    uint32_t time_ms(){

		std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
		uint32_t time_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end-time_begin).count();

		return time_now_ms;
    }


    uint32_t time_us(){

		std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
		uint32_t time_now_us = std::chrono::duration_cast<std::chrono::microseconds>(time_end-time_begin).count();

		return time_now_us;
    }

	void delay_ms(uint32_t wait_ms){

		std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
	}

	void delay_us(uint32_t wait_us){

		std::this_thread::sleep_for(std::chrono::microseconds(wait_us));
	}

  private:

	CircularBuffer<uint8_t>* cbuffer;

	enum state_enum {CONNECTED, DISCONNECTED}; 
	int current_state;
	int last_state;

	serial::Serial serial;

	const char* serial_port;
	int baudrate;
	int timeout;
	int reconnection_time;

	time_t current_time, last_time;

	std::chrono::steady_clock::time_point time_begin = std::chrono::steady_clock::now();

	void switchState(int state){

		if (state != current_state){
			last_state = current_state;
			current_state = state;
		}

	}

};



#endif