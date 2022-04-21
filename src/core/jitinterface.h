#ifndef JITINTERFACE_H
#define JITINTERFACE_H


class JitInterface: public Jitlog { 

  public:

    JitInterface(){};
    ~JitInterface(){};

    virtual bool write(uint8_t data){

      return 0;
    };

    virtual uint32_t available_write(){

      return 0;
    };
  
    virtual uint8_t read(){

      return 0;
    };

    virtual uint32_t available_read(){

      return 0;
    };

    virtual uint32_t time_ms(){

      return 0;
    }

    virtual uint32_t time_us(){

      return 0;
    }

    virtual void delay_ms(uint32_t wait_ms){

    };

    virtual void delay_us(uint32_t wait_us){

    };

  private:

};


#endif