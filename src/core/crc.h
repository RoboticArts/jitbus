#ifndef CRC_H
#define CRC_H


class CRC { 

  public:

    CRC(){};
    ~CRC(){};

    static bool check_crc16(uint8_t* buffer, uint32_t size){

        return true;
    }

    static uint16_t compute_crc16(uint8_t* buffer, uint32_t size){

        return 0;
    }


  private:

};

#endif