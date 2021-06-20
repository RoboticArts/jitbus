#ifndef SERIAL_JITBUS_H
#define SERIAL_JITBUS_H

#define JITLOG_SIZE 1000

#include <time.h>
#include "serial/serial.h"
#include "core/jitcore.h"

class SerialJitbus: public Jitcore { 

  public:
	
    SerialJitbus(){

		enable_color = true;
		time(&last_time);
		time(&current_time);
		last_state = CONNECTED;
		current_state = DISCONNECTED;

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

    void write(uint8_t* buffer, int bytes_to_write){

		if (connected()){

			try{

				serial.write(buffer, bytes_to_write);
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

    }

    void read(CircularBuffer<uint8_t>* cbuffer,  int bytes_to_read){

		if (connected()){

			try{
				
				std::string incoming_bytes;
				size_t available_bytes = serial.available();
			
//printf("%zu\n", serial.available());

				if (available_bytes > 0){
					
					if (bytes_to_read > available_bytes){
					
						bytes_to_read = available_bytes;
					}


					incoming_bytes = serial.read(bytes_to_read);
					
					for (int i = 0; i<incoming_bytes.length(); i++){
						
						cbuffer->push(incoming_bytes[i]);
					}

				} 
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

	void print_log(const char* message){

		printf("%s", message);

	}

  private:

	enum state_enum {CONNECTED, DISCONNECTED}; 
	int current_state;
	int last_state;

	serial::Serial serial;

	const char* serial_port;
	int baudrate;
	int timeout;
	int reconnection_time;

	time_t current_time, last_time;

	void switchState(int state){

		if (state != current_state){
			last_state = current_state;
			current_state = state;
		}

	}

};



#endif