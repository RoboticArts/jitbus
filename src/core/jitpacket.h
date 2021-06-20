#ifndef JITPACKET_H
#define JITPACKET_H

/*
*   ||___________________HEADER___________________||_____DATA____||__________TAIL_________||
*   || START FRAME | TOTAL LENGTH | PACKET INDEX  || PACKET DATA ||   CRC     | END FRAME ||
*   ||   uint16_t  |   uint32_t   |   uint16_t    ||   uint8_t*  || uint16_t  |  uint16_t ||       
*   ||    0xFFFF   |     ---      |     ---       ||     ---     ||   ---     |   0xFE7F  ||
*/    

#define PROTOCOL_START_FRAME 0xFFFF
#define PROTOCOL_END_FRAME 0xFE7F
#define PROTOCOL_HEADER_LENGTH 8
#define PROTOCOL_TAIL_LENGTH 4 


class JitPacket{ 

    public:

        uint8_t buffer[100];
        JitPacket(){};
        ~JitPacket(){};
    
        // --------------- BUILD PACKET  ---------------- //

        template<typename Type>
        void buildPacket(const Type& data, uint16_t data_id){

            uint32_t data_length = sizeof(Type);
            buildHeader(data_id, data_length);
            buildData(data, data_length);
            buildTail(data_length);

        };

        void buildHeader(uint16_t data_id, uint32_t data_length){
            
            // Start frame
            uint16_t start_frame = PROTOCOL_START_FRAME;
            buffer[0] = start_frame >> 8 & 0xFF;
            buffer[1] = start_frame & 0xFF;
            
            // Total length
            uint32_t total_length = PROTOCOL_HEADER_LENGTH + data_length + PROTOCOL_TAIL_LENGTH;
            this->total_length = total_length;
            buffer[2] = total_length >> 24 & 0xFF;
            buffer[3] = total_length >> 16 & 0xFF;
            buffer[4] = total_length >> 8 & 0xFF;
            buffer[5] = total_length & 0xFF;

            // Data id
            buffer[6] = data_id >> 8 & 0xFF;
            buffer[7] = data_id & 0xFF;
            

        }

        template<typename Type>
        void buildData(const Type& data,  uint32_t data_length){

            // Data
            uint8_t* data_ptr = (uint8_t*)&data;

            for (uint32_t i = PROTOCOL_HEADER_LENGTH; i < PROTOCOL_HEADER_LENGTH + data_length; i++){
                buffer[i] = *data_ptr;
                data_ptr++;
            }

        }

        void buildTail(uint32_t data_length){

            // CRC
            uint16_t crc = calculate_crc(data_length);
            buffer[PROTOCOL_HEADER_LENGTH + data_length + 0] = crc >> 8 & 0xFF;
            buffer[PROTOCOL_HEADER_LENGTH + data_length + 1] = crc & 0xFF;

            // End frame
            uint16_t end_frame = PROTOCOL_END_FRAME;
            buffer[PROTOCOL_HEADER_LENGTH + data_length + 2] = end_frame >> 8 & 0xFF;
            buffer[PROTOCOL_HEADER_LENGTH + data_length + 3] = end_frame & 0xFF;

        }

        int calculate_crc(uint32_t data_length){

            return 0;
        }
        

        // --------------- FIND PACKET  ---------------- //

        bool findPacket(CircularBuffer<uint8_t>* cbuffer){
            
            bool packet_found = false;
            int find = START_FRAME;
  /*          
            for(int i = 0; i<cbuffer->size(); i++){
                Serial.println(cbuffer->index(i));
            }
*/
            //printf("%d\n", cbuffer->size() );
            while(cbuffer->size() > 0 && packet_found == false){

                switch(find){

                    case START_FRAME: 
                        
                        if (findStartFrame(cbuffer) == true){
                    
                            find = LENGTH_FRAME;

                        }

                        else{

                            find = START_FRAME;

                        }
                        
                        break;

                    case LENGTH_FRAME:

                        if (findLengthFrame(cbuffer) == true){

                            find = END_FRAME;

                        }
                        
                        else{

                            find = START_FRAME;

                        }

                        break;

                    case END_FRAME:

                        if (findEndFrame(cbuffer) == true){

                            find = CRC_FRAME;

                        }

                        else{

                            find = START_FRAME;

                        }

                        break;

                    case CRC_FRAME:
                        
                        if (findCrcFrame(cbuffer) == true){

                            packet_found = true;

                        }

                        else{

                            find = START_FRAME;

                        }

                        break;

                };

            }   

            return packet_found;

        }

        bool findStartFrame(CircularBuffer<uint8_t>* cbuffer){

            bool found = false;

            uint16_t start_frame = 0;
            start_frame = start_frame | cbuffer->index(0) << 8;
            start_frame = start_frame | cbuffer->index(1) << 0;

            if (start_frame == PROTOCOL_START_FRAME){

                found = true;
            }
            else{
                found = false;
                cbuffer->pop();
                //printf("start frame failed\n");
            }

            return found;
        }

        bool findLengthFrame(CircularBuffer<uint8_t>* cbuffer){

            bool found = false;

            uint32_t total_length = 0;
            total_length = total_length | cbuffer->index(2 + 0) << 24;  
            total_length = total_length | cbuffer->index(2 + 1) << 16;    
            total_length = total_length | cbuffer->index(2 + 2) << 8;    
            total_length = total_length | cbuffer->index(2 + 3) << 0; 

            if (total_length <= cbuffer->free_size() && total_length > PROTOCOL_HEADER_LENGTH + PROTOCOL_TAIL_LENGTH){
                found = true;
                this->total_length = total_length;
            }
            else{
                found = false;
                cbuffer->pop();
                printf("length frame failed\n");
            }

            return found;
        }

        bool findEndFrame(CircularBuffer<uint8_t>* cbuffer){

            bool found = false;
           
            uint16_t end_frame = 0;
            end_frame = end_frame | cbuffer->index((this->total_length-1) - 1) << 8;
            end_frame = end_frame | cbuffer->index((this->total_length-1) - 0) << 0;

            if (end_frame == PROTOCOL_END_FRAME){

                found = true;
            }
            else{
                found = false;
                cbuffer->pop();
                printf("end frame failed\n");
            }

            return found;
        }

        bool findCrcFrame(CircularBuffer<uint8_t>* cbuffer){
            
            bool found = false;

            int data_from = PROTOCOL_HEADER_LENGTH;
            int data_to = data_from + (this->total_length - PROTOCOL_TAIL_LENGTH);
            int crc_from =  (this->total_length - PROTOCOL_TAIL_LENGTH) + 1;
            int crc_to = crc_from + 2; 

            if (check_crc16_from_cbuffer(cbuffer, data_from, data_to, crc_from, crc_to) == true){

                found = true;
                this->id = this->id | cbuffer->index(6) << 8;
                this->id = this->id | cbuffer->index(7) << 0;
                //printf("frame OK\n");
            }
            else{
                found = false;
                cbuffer->pop();
                //printf("crc frame failed\n");
            }
            
            found = true; //  Remove when CRC is implemented! <------------------------------------------------------
            return found;

        }


        bool check_crc16_from_cbuffer(CircularBuffer<uint8_t>* cbuffer, int data_from, int data_to, int crc_from, int crc_to){

            return true;
        }


        uint32_t packetLength(){

            return this->total_length;
        }

        int packetId(){

            return this->id;
        }

        // --------------- PARSE PACKET  ---------------- //

        template<typename Type>
        void parsePacket(const Type& data, CircularBuffer<uint8_t>* cbuffer, int packet_length){
            
            int received_data_length = packet_length - PROTOCOL_HEADER_LENGTH - PROTOCOL_TAIL_LENGTH;
            int current_data_length = sizeof(Type);

            int data_from = PROTOCOL_HEADER_LENGTH;
            int data_to;

            if(received_data_length > current_data_length){
                
                data_to = data_from + current_data_length - 1;

            }

            else{

                data_to = data_from + received_data_length - 1;
            }

            uint8_t* data_ptr = (uint8_t*)&data;

            for (int i = data_from; i <= data_to; i++){
               
                *data_ptr = cbuffer->index(i);
                data_ptr++;
            }

            cbuffer->pop_until_index(packet_length);
        }


    private:

        uint32_t total_length = 0;
        int id = 0;

        enum process {START_FRAME, LENGTH_FRAME, ID_FRAME, DATA_FRAME, CRC_FRAME, END_FRAME};
               

};

#endif