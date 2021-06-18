#ifndef CBUFFER_H
#define CBUFFER_H


template <typename Type>
class CircularBuffer{ 
  
    public:

        CircularBuffer(uint32_t size){

            pointerWrite = 0;
            pointerRead = 0;
            flagEOV = false;
            buffer_size = size;
            buffer = new uint8_t [buffer_size];

        };

        ~CircularBuffer(){};


        Type pop(){

            Type data = 0;
            
            if(pointerRead - pointerWrite > 0){
                flagEOV = false;
            }
            
                
            data = buffer[pointerRead];
            pointerRead++;

            if(pointerRead >= buffer_size){
                
                pointerRead = 0;
            }
            
            return data;
        }


        void push(Type data){
  
  
            if(flagEOV == false){
                
                buffer[pointerWrite] = data;
                pointerWrite++;
            }
            
            else{
                
                if(pointerWrite == pointerRead ){
                    pop();
                }
                
                buffer[pointerWrite] = data;
                pointerWrite++;
                
            }
            
            if(pointerWrite == pointerRead-1){

                flagEOV = true;

            }

            if(pointerWrite >= buffer_size){
                
                pointerWrite = 0;
                flagEOV = true; 
                
            }
  
        }


        Type at(uint32_t n){
            return buffer[n];
        }

        Type index(uint32_t n){

            uint32_t position = pointerRead+n;

            if (position > buffer_size-1){

                position = (pointerRead+n) - buffer_size;
            }

            return buffer[position];
        }

        void pop_until_index(uint32_t pops){

            for(uint32_t i = 0; i<pops; i++){
                pop();
            }

        }

        uint32_t max_size(){

            return buffer_size;
        }

        uint32_t size(){

            uint32_t size;

			if((pointerWrite == pointerRead) && (flagEOV == false)){
				size = 0;
			}
			
			if((pointerWrite == pointerRead) && (flagEOV == true)){
				size = buffer_size;
			}
						
				
			if(pointerWrite > pointerRead){
				
				size = pointerWrite - pointerRead;
			}
		
			if(pointerWrite < pointerRead){
			
				size =  (buffer_size - pointerRead) + pointerWrite;
			}
	
	        return size;
        }

        uint32_t free_size(){

            return buffer_size - size();
        }


  /*
        uint8_t& operator[](int index){

            return buffer[index];
        }
  */

    private:

        uint32_t pointerWrite;
        uint32_t pointerRead;
        uint8_t* buffer;
        uint32_t buffer_size;
        bool flagEOV;


};



#endif