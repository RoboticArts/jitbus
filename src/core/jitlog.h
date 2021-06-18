#ifndef JITLOG_H
#define JITLOG_H

#include <stdarg.h>

#ifndef JITLOG_SIZE
    #define JITLOG_SIZE 100
#endif

class Jitlog { 

  public:

    bool enable_color;

    Jitlog(){}

    ~Jitlog(){}

    #ifndef JITBUS_DISABLE_LOG


        virtual void print_log(const char* message){

        }

        void print_debug(const char *format, ...) {
            
            va_list args;

            va_start(args, format);
            print_message(DEBUG, format, args);
            va_end(args);

        }

        void print_info(const char *format, ...) {
            
            va_list args;

            va_start(args, format);
            print_message(INFO, format, args);
            va_end(args);

        }


        void print_warn(const char *format, ...) {
            
            va_list args;
            
            va_start(args, format);
            print_message(WARN, format, args);
            va_end(args);

        }

        void print_error(const char *format, ...) {
            
            va_list args;

            va_start(args, format);
            print_message(ERROR, format, args);
            va_end(args);

        }

        void print_fatal_error(const char *format, ...) {
            
            va_list args;

            va_start(args, format);
            print_message(FATAL_ERROR, format, args);
            va_end(args);

        }

    #else
        
        void print_debug(const char *format, ...){}
        void print_info(const char *format, ...){}
        void print_warn(const char *format, ...){}
        void print_error(const char *format, ...){}
        void print_fatal_error(const char *format, ...){}
    
    #endif

  private:

    #ifndef JITBUS_DISABLE_LOG

        enum level {DEBUG, INFO, WARN, ERROR, FATAL_ERROR};

        void print_message(int level, const char* format, va_list args){

            char message[JITLOG_SIZE];
            vsprintf(message, format, args);
            color_message(level);
            print_log(message);
            print_log("\r\n");
            color_message(-1);

        }


        void color_message(int level){

            if (enable_color == true){

                switch(level){
                    
                    case DEBUG: print_log("\033[0;32mDEBUG: "); break;
                    case INFO: print_log("\033[0;37mINFO: "); break;
                    case WARN: print_log("\033[0;33mWARN: "); break;
                    case ERROR: print_log("\033[0;31mERROR: "); break;
                    case FATAL_ERROR: print_log("\033[0;31mFATAL_ERROR: "); break;
                    default: print_log("\033[0;37m"); break;
                }
            }

        }

    #endif


/*
	void print_error(const char *format, ...) {
  		
		printf("\033[1;31mERROR: ");
		va_list args;
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
		printf("\033[0m");
	}

	void print_warn(const char *format, ...) {

		printf("\033[1;33mWARN: ");
		va_list args;
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
		printf("\033[0m");
	}

	void print_info(const char *format, ...) {

  		printf("\033[0mINFO: ");
		va_list args;
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
		printf("\033[0m");
		  
	}
*/
    

};

#endif
