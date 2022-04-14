#ifndef JITSTREAM_H
#define JITSTREAM_H

// all: Review all debug, info, warn and print messages
// all: check variable sizes
// crc: CRC CCIT_ZERO
// cobs: Remove auxiliar buffer for cobs, package will be built with enough spaces, add start and end frame. Keep length warning
// jitstream: Fix attempts when start data is not received after X attempts
// jitstream: add bool inside the write function to know if data is sent
// jitstream: improve bool inside the read function, rename types
// jitstream: test and update sendPacketBlocking
// jitstream: Add clear packet function when packet is not consumed, it will cause packet block
// jitstream: increase precision time to publish to more than 500 hz. Add overflow counter
// linux/SerialJitbus: add virtual time function to be overloaded on ros programs
// SerialJitbus: Add set_print_level out of the jitbus library
// SerialJitbus: 
// arduino/SerialJitbus: remove writeAvailable because is not available in all devices. Intead of it, use return of write
// arduino/SerialJitbus: check how to manage Stream->Serial and disable write and read functions when port is closed
// arduino/SerialJitbus: check if circular buffer can be removed. Check programs with low loop frequency
// raspberry/SerialJitbus: check if serial library is compatible with native serial on raspberry pi, or find a library

class JitStream: public JitPacket{ 

  public:

    JitStream(){};
    ~JitStream(){};


    // ------------------------------------------------------------------------------- // 

    //                             PUBLIC READING METHODS                              //

    // ------------------------------------------------------------------------------- // 

    bool availablePacket(){

        do {

            switch(state_read){

                case RD_FIND_START: 


                    if (packet_detected == true){

                        packet_detected = false;
                        print_error("Packet FAIL");
                    }

                    if (findStart() == true){

                        state_read = RD_SAVE_DATA;
                        packet_detected = true;
                        //print_info("New possible packet found");

                    }
                    else{

                        state_read = RD_FIND_START;
                    }


                    break;

                case RD_SAVE_DATA: 
                {

                int status = saveData();

                    if(status == 1){
                        
                        // Nothing to do, wait for next data available
                    }
    
                    else if (status == 2){

                        state_read = RD_PROCESS_DATA;
                    }

                    else if (status == 3){

                        state_read = RD_FIND_START;
                    }
                    
                    break;
                }

                case RD_PROCESS_DATA:

                    if(processData() == true){

                        packet_available = true;
                        packet_detected = false;
                        state_read = RD_WAIT_PACKET_READING;
                        print_info("Packet OK");
                    }

                    else{

                        state_read = RD_FIND_START;
                    }

                    break; 
        
            
                case RD_WAIT_PACKET_READING:

                    if(waitPacketReading() == true){

                        state_read = RD_WAIT_PACKET_READING;
                    }

                    else{

                        state_read = RD_FIND_START;
                    }

                    break; 

            };

        } while (state_read != RD_FIND_START && state_read !=RD_SAVE_DATA );
     
        return packet_available;       
    
    }


    template<typename Type>
    bool readPacket(const Type& data, uint16_t data_id){

        bool is_received = false;

        if (packet_available == true){

            if (data_id == JitPacket::packetId(buffer_read)){
          
                JitPacket::parsePacket(data, buffer_read, buffer_read_size);
                packet_available = false;
                buffer_read_size = 0;
                is_received = true;
            }

        }

        return is_received;
    }
   


    // ------------------------------------------------------------------------------- // 

    //                             PUBLIC WRITING METHODS                              //

    // ------------------------------------------------------------------------------- // 

    template<typename Type>
    bool writePacket(const Type& data, uint16_t data_id){

        bool is_packet_sent = false;
        // print_info("state %d", state_write);
        if (state_write == NEW_PACKET){

            switchWriteState(BUILD_DATA_FRAME);
        }

        if (state_write == BUILD_DATA_FRAME){

            if(buildNewPacket(data, data_id) == true){
                
                switchWriteState(WRITE_START_FRAME);
            }
            else{

                switchWriteState(NEW_PACKET);
            }
        }

        if (state_write == WRITE_START_FRAME){

            if (write(JitPacket::startFrame()) == true){
                
                switchWriteState(WRITE_DATA_FRAME);

            }   
            else{

                state_write = WAIT_BYTES;
            }
        }

        if (state_write == WRITE_DATA_FRAME){

            if (writeThisPacket() == true){

                switchWriteState(WRITE_END_FRAME);
            }
            else{

                switchWriteState(WAIT_BYTES);
            }
        }

        if (state_write == WRITE_END_FRAME){
            
            if (write(JitPacket::endFrame()) == true){

                switchWriteState(COMPLETE_PACKET);
            }

            else{

                switchWriteState(WAIT_BYTES);
            }
        }

        if (state_write == COMPLETE_PACKET){

            is_packet_sent = true;
            switchWriteState(NEW_PACKET);
        }

        if (state_write == WAIT_BYTES){
            
            if (isWrittenBytesAvailable() == true){

                switchWriteState(last_state_write);
            }
            else{

                switchWriteState(WAIT_BYTES);
            }

        }

        return is_packet_sent;
    }


    template<typename Type>
    void writePacketHz(const Type& data, uint16_t data_id, uint32_t& time, float frequency){

        if (time_ms() - time >= 1000/frequency){
            
            if (writePacket(data, data_id) == true){
                time = time_ms();
            }
           
        }
    }


    template<typename Type>
    void writePacketBlocking(const Type& data, uint16_t data_id){

        //It finishes sending the last package if it was sending
        if (index_written > 0){
            
            while (index_written > 0){

                writePacket(data, data_id);
            
            }
        }

        // Send the new packet
        writePacket(data, data_id);

        // Wait for the new package to be sent
        while (index_written > 0){
         
            writePacket(data, data_id);
        }

    }

  private: 

    // ------------------------------------------------------------------------------- // 

    //                            PRIVATE READING METHODS                              //

    // ------------------------------------------------------------------------------- // 


    uint8_t buffer_read[JITBUS_BUFFER_SIZE];
    uint32_t buffer_read_size = 0;

    int packet_id = 0;

    bool packet_available = false;
    bool packet_detected = false;

    enum state_read_enum {RD_FIND_START, RD_SAVE_DATA, RD_PROCESS_DATA, RD_WAIT_PACKET_READING};
    int state_read = RD_FIND_START;


    bool findStart(){

        bool success = false;
        uint32_t attempts = 0;
        uint32_t available_data = available_read();

        buffer_read_size = 0;

        if (available_data > 0){
            
            print_debug("jitcore::findStart -> Available data: %d", available_data);

        }

        while (available_data > 0){
    
            uint8_t read_byte = read();

            if (read_byte== JitPacket::startFrame()){

                success = true;
                print_debug("jitcore::findStart -> Start frame detected in %d position", available_data);
                available_data = 0;
            }

            else{
                print_warn("jitcore::findStart -> Start frame not found in this byte  dec: %d  hex: %X char %c, needed %d", read_byte, read_byte, read_byte, JitPacket::startFrame());

                /*
                if (attempts == JITBUS_BUFFER_SIZE/2){

                    print_warn("jitcore::findStart -> Start frame not found after %d attempts", JITBUS_BUFFER_SIZE/2);
                }
                */
                attempts++;
                success = false;
                available_data--;
            }

        }

        return success;
    }

    int saveData(){
        
        // 1 -> SAVE_DATA
        // 2 -> CHECK_DATA
        // 3 -> FIND_START

        int status = 1;
        uint32_t available_data = available_read();

        if (available_data > 0){
            
            print_debug("jitcore::saveData -> Available data: %d", available_data);

        }

        bool exit = false;

        while(available_data > 0 && exit == false){
 
            if (buffer_read_size >= JITBUS_BUFFER_SIZE - 1){
                
                status = 3;
                print_debug("jitcore::saveData -> Buffer is full, possible packet discarted");
                exit = true;
            }
            else{

                uint8_t read_byte = read();
                available_data--;

                if (read_byte == JitPacket::startFrame()){
                    
                    status = 3;
                    print_debug("jitcore::saveData --> Start frame detected again, possible packet corrupted");
                    exit = true;
                }

                else if (read_byte == JitPacket::endFrame()){

                    status = 2;
                    print_debug("jitcore::saveData -> End frame detected");
                    exit = true;
                    
                }
                else{

                    buffer_read[buffer_read_size] = read_byte; 
                    //print_info("%X", buffer_read[buffer_read_size]);
                    buffer_read_size++;
                }

                if (available_data <= 0 && exit != true){
                    
                    status = 1;
                    print_debug("jitcore::saveData -> Possible packet needs more bytes, waiting for available bytes");
                   
                    exit = true;
                }

            }

        }

        return status;
    }


    bool processData(){

        bool success = false;

        if (JitPacket::processPacket(buffer_read, buffer_read_size) == true){ 
            
            success = true;
        }

        else{

            success = false;
        }

        return success;
    }

    bool waitPacketReading(){

        bool success = false;

        if (packet_available == true){

            success = false;
        }
        else{
            success = true;
        }

        return success;
    }


    // ------------------------------------------------------------------------------- // 

    //                            PRIVATE WRITING METHODS                              //

    // ------------------------------------------------------------------------------- // 

    uint8_t buffer_write[JITBUS_BUFFER_SIZE];
    uint32_t buffer_write_size;
    uint32_t index_written = 0;
    enum state_write_enum {NEW_PACKET, BUILD_DATA_FRAME, WRITE_START_FRAME, WRITE_DATA_FRAME, WRITE_END_FRAME, WAIT_BYTES, COMPLETE_PACKET};
    int state_write = NEW_PACKET;
    int last_state_write = NEW_PACKET;
    bool write_busy = false;
    uint16_t write_id = 0;

    template<typename Type>
    bool buildNewPacket(const Type& data, uint16_t data_id){

        bool new_packet = false;

        buffer_write_size = JitPacket::buildPacket(buffer_write, data, data_id);
        
        if (buffer_write_size > 0){

            new_packet = true;
            //print_debug("Sending new packet...");
        }
        else{
            print_warn("jitcore::sendPacket -> Wrong buffer write size, length is %d ", buffer_write_size);
            index_written = 0;
            new_packet = false;
        }

        return new_packet;
    }

    bool writeThisPacket(){

        bool is_packet_written = false;
        uint32_t available_write_bytes = available_write();
        bool success = true;
    
        while(available_write_bytes > 0 && index_written < buffer_write_size && success == true){
            
            success = write(buffer_write[index_written]);
            //print_info("%X", buffer_write[index_written]);
            if (success == true){
                index_written++;
                available_write_bytes = available_write();
            }

        }

        if (index_written >= buffer_write_size){

            index_written = 0;
            is_packet_written = true;
            //print_debug("Packet sent completly");
        }
        else{
            is_packet_written = false;
            print_debug("jitcore::sendPacket -> Buffer TX full because there is %d available bytes for write", available_write_bytes);
            print_debug("X Waiting for available %d write bytes", buffer_write_size - ((index_written-1) + 1) );
        }

        return is_packet_written;
    }

    bool isWrittenBytesAvailable(){

        bool available_bytes_to_write = false;


        if (available_write() > 0){

            available_bytes_to_write = true;
            print_debug("There are new %d write bytes available", available_write());
        }

        else{
            
            available_bytes_to_write = false;
            //print_debug("Waiting for available %d write bytes", buffer_write_size - ((index_written-1) + 1));
        }

        return available_bytes_to_write;
    }

    void switchWriteState(int state){

		if (state != state_write){
			last_state_write = state_write;
			state_write = state;
		}

	}


    // ------------------------------------------------------------------------------- // 

    //                                     END METHODS                                 //

    // ------------------------------------------------------------------------------- // 



};


#endif