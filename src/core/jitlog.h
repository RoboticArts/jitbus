#ifndef JITLOG_H
#define JITLOG_H

#include <stdarg.h>

#ifndef JITLOG_SIZE
    #define JITLOG_SIZE 200
#endif

class Jitlog { 

  public:

    bool enable_color;
    enum level {FATAL_ERROR, ERROR, WARN, INFO, DEBUG};

    Jitlog(){}

    ~Jitlog(){}

    #ifndef JITBUS_DISABLE_LOG

        void set_print_level(int level){

            _print_level = level;
        }

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
    
    int _print_level = DEBUG;

    #ifndef JITBUS_DISABLE_LOG

        void print_message(int level, const char* format, va_list args){

            if (level <= _print_level){

                char message[JITLOG_SIZE];
                vsprintf(message, format, args);
                color_message(level);
                print_log(message);
                print_log("\r\n");
                color_message(-1);
            }
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

            else{

                switch(level){
                    
                    case DEBUG: print_log("DEBUG: "); break;
                    case INFO: print_log("INFO: "); break;
                    case WARN: print_log("WARN: "); break;
                    case ERROR: print_log("ERROR: "); break;
                    case FATAL_ERROR: print_log("FATAL_ERROR: "); break;
                    default: print_log(""); break;
                }

            }

        }

    #endif

    

};

#endif
