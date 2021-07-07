#ifndef JITPACKET_H
#define JITPACKET_H

/*
*   ||___________________HEADER___________________||_____DATA____||__________TAIL_________||
*   || START FRAME | TOTAL LENGTH | PACKET INDEX  || PACKET DATA ||   CRC     | END FRAME ||
*   ||   uint16_t  |   uint32_t   |   uint16_t    ||   uint8_t*  || uint16_t  |  uint16_t ||       
*   ||    0xFFFF   |     ---      |     ---       ||     ---     ||   ---     |   0xFE7F  ||
*/    

#define PROTOCOL_START_FRAME 0xAF
#define PROTOCOL_END_FRAME 0xFE
#define PROTOCOL_DATA_ID_LENGTH uint32_t(sizeof(uint16_t))
#define PROTOCOL_CRC_LENGTH uint32_t(sizeof(uint16_t))

class JitPacket: public JitInterface { 

    public:

        JitPacket(){};

        ~JitPacket(){};
    
        // --------------- BUILD PACKET  ---------------- //

        template<typename Type>
        uint32_t buildPacket(uint8_t* buffer, const Type& data, uint16_t data_id){
            
            uint32_t cobs_length;
            uint32_t data_length = sizeof(Type);
            uint32_t packet_length = PROTOCOL_DATA_ID_LENGTH + data_length + PROTOCOL_CRC_LENGTH;

            // Data id
            buffer[0] = data_id >> 8 & 0xFF;
            buffer[1] = data_id & 0xFF;

            // Data
            uint8_t* data_ptr = (uint8_t*)&data;

            for (uint32_t i = PROTOCOL_DATA_ID_LENGTH; i < 2 + data_length; i++){
                buffer[i] = *data_ptr;
                data_ptr++;
            }

            // CRC
            uint16_t crc = CRC::compute_crc16(buffer, packet_length);
            buffer[PROTOCOL_DATA_ID_LENGTH + data_length + 0] = crc >> 8 & 0xFF;
            buffer[PROTOCOL_DATA_ID_LENGTH + data_length + 1] = crc & 0xFF;

            cobs_length = encodePacket(buffer, packet_length);
            
            return cobs_length;
        };


        uint32_t encodePacket(uint8_t* buffer, uint32_t buffer_length){

            uint8_t buffer_aux[JITBUS_BUFFER_SIZE];
            uint32_t total_bytes, total_bytes_aux;

            total_bytes_aux = COBS::encode(buffer, buffer_length, buffer_aux, PROTOCOL_START_FRAME);
            total_bytes = COBS::encode(buffer_aux, total_bytes_aux, buffer, PROTOCOL_END_FRAME);
        
            return total_bytes;
        }


        // --------------- PROCESS PACKET ---------------- //

        bool processPacket(uint8_t* buffer, uint32_t& buffer_length){
            
            bool packet_status = false;
/*
            for(uint32_t i = 0; i<buffer_length; i++){

                print_debug("%X", buffer[i]);
            }
*/
            buffer_length = decodePacket(buffer, buffer_length);

            if (buffer_length > 0){

                if (CRC::check_crc16(buffer, buffer_length) == true){

                    packet_status = true;
                    print_debug("jitpacket::processPacket -> CRC ok");
                    
                }
                else{
                    packet_status = false;
                    print_debug("jitpacket::processPacket -> CRC failed");
                }            
            } 
            else{
                packet_status = false;
                print_debug("jitpacket::processPacket -> Decode failed because length does not match data, malformed");
                   
            }   

            return packet_status;

        }

        bool check_crc16(uint8_t* buffer, uint32_t buffer_length){

            return true;
        }

        uint32_t decodePacket(uint8_t* buffer, uint32_t buffer_length){

            uint8_t buffer_aux[JITBUS_BUFFER_SIZE];
            uint32_t total_bytes = 0, total_bytes_aux;

            total_bytes_aux = COBS::decode(buffer, buffer_length, buffer_aux,  PROTOCOL_END_FRAME);
            total_bytes = COBS::decode(buffer_aux, total_bytes_aux, buffer,  PROTOCOL_START_FRAME);

            return total_bytes;
        }

        // --------------------- OTHERS -------------------- //

        uint8_t startFrame(){

            return PROTOCOL_START_FRAME;
        }

        uint8_t endFrame(){

            return PROTOCOL_END_FRAME;
        }

        uint16_t packetId(const uint8_t* buffer){

            uint16_t packet_id = 0;

            packet_id = packet_id | buffer[1] << 0;
            packet_id = packet_id | buffer[0] << 8;

            return packet_id;
        }

        // --------------- PARSE PACKET  ---------------- //

        template<typename Type>
        void parsePacket(const Type& data, uint8_t* buffer, uint32_t buffer_length){
    
            uint8_t* data_ptr = (uint8_t*)&data;

            for (uint32_t i = PROTOCOL_DATA_ID_LENGTH; i < buffer_length-PROTOCOL_CRC_LENGTH; i++){
               
                *data_ptr = buffer[i];
                data_ptr++;
            }
   
        }


    private:

      

};

#endif