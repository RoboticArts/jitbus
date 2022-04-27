#ifdef __linux__

#include <boost/python.hpp>
#include "../../core/jitcore.h"

class JitbusBinding: public Jitcore {

  public:

	JitbusBinding(){}

	~JitbusBinding(){}
	
	uint32_t custom_data_length;

	// Virtual function from jitpacket

	uint32_t setLength(uint32_t data_sizeof){

		return custom_data_length;
	}

    bool available_py(){
		
        return available();
    }

 	bool sendPacket_py(const char &data, uint8_t data_length, uint16_t data_id){

		custom_data_length = data_length;
		return sendPacket(data, data_id);
	}

	bool receivePacket_py(const char &data, uint16_t data_id){
		
		return receivePacket(data, data_id);
	}

	bool sendPacketBlocking_py(const char &data, uint8_t data_length, uint16_t data_id, uint32_t timeout){

		custom_data_length = data_length;
		return sendPacketBlocking(data, data_id, timeout);
    }

	boost::python::tuple sendPacketHz_py(const char &data, uint8_t data_length, uint16_t data_id, uint32_t time, float frequency){

		custom_data_length = data_length;
        bool is_sent = sendPacketHz(data, data_id, time, frequency);
		return boost::python::make_tuple(time, is_sent);
    }

	void print_debug_py(std::string message){
		
		print_debug("%s", message.c_str());
	}

	void print_info_py(std::string message){
		
		print_info("%s", message.c_str());
	}

	void print_warn_py(std::string message){
		
		print_warn("%s", message.c_str());
	}

	void print_error_py(std::string message){
		
		print_error("%s", message.c_str());
	}

	void print_fatal_error_py(std::string message){
		
		print_fatal_error("%s", message.c_str());
	}

	void print_set_level_py(int level){

		set_print_level(level);
	}

	void print_enable_color_py(bool enabled){
		
		this->enable_color = enabled;
	}

/*
	boost::python::tuple receivePacketPy(){
		
		bool success = true;
		uint16_t data_id = 4; 

		std::string data("");
		data.push_back(0x2);
		data.push_back('\0');
		data.push_back('\0');
		data.push_back('\0');
		data.push_back(0x3);
		data.push_back('\0');
		data.push_back('\0');
		data.push_back('\0');
		data.push_back(0x5);
		data.push_back('\0');
		data.push_back('\0');
		data.push_back('\0');
		
		//for (int i = 0; i< 12; i++)
		//	printf("%X ", data[i]);

		//receivePacket(data, data_id);

		return boost::python::make_tuple(success, data, data_id);
	}
*/

};



#include <boost/python.hpp>
using namespace boost::python;

struct JitbusBindingWrap : JitbusBinding, wrapper<JitbusBinding>
{

    uint8_t read() override
    {

        if (override f = this->get_override("read"))
        {
            return this->get_override("read")();
        } 
        else
        {
            return JitbusBinding::read();
        }

    }

	bool write(uint8_t data) override
    {

        if (override f = this->get_override("write"))
        {
        	return this->get_override("write")(data);
        } 
        else
        {
        	return JitbusBinding::write(data);
        }

    }

	uint32_t available_read() override
    {

        if (override f = this->get_override("available_read"))
        {
            return this->get_override("available_read")();
        } 
        else
        {
            return JitbusBinding::available_read();
        }

    }

	uint32_t available_write() override
    {

        if (override f = this->get_override("available_write"))
        {
            return this->get_override("available_write")();
        } 
        else
        {
            return JitbusBinding::available_write();
        }

    }

	uint32_t time_ms() override
    {

        if (override f = this->get_override("time_ms"))
        {
        	return this->get_override("time_ms")();
        } 
        else
        {
        	return JitbusBinding::time_ms();
        }

    }

	uint32_t time_us() override
    {

        if (override f = this->get_override("time_us"))
        {
			return this->get_override("time_us")();
        }
        else
        {
			return JitbusBinding::time_us();
        }

    }

	void delay_ms(uint32_t wait_ms) override
	{

	}

	void delay_us(uint32_t wait_us) override
	{

	}

	void print_log(const char* message) override
    {

        if (override f = this->get_override("print_log"))
        {
        	this->get_override("print_log")(message);
        } 
        else
        {
        	JitbusBinding::print_log(message);
        }

    }

};


BOOST_PYTHON_MODULE(_jitbus_binding_so)
{
    class_<JitbusBindingWrap, boost::noncopyable>("JitbusBinding")
        .def("read", &JitbusBinding::read)
		.def("write", &JitbusBinding::write)
		.def("available_read", &JitbusBinding::available_read)
		.def("available_write", &JitbusBinding::available_write)
		.def("time_ms", &JitbusBinding::time_ms)
		.def("time_us", &JitbusBinding::time_us)
		.def("delay_ms", &JitbusBinding::delay_ms)
		.def("delay_us", &JitbusBinding::delay_us)
		.def("print_log", &JitbusBinding::print_log)
		.def("print_debug_py", &JitbusBinding::print_debug_py)
		.def("print_info_py", &JitbusBinding::print_info_py)
		.def("print_warn_py", &JitbusBinding::print_warn_py)
		.def("print_error_py", &JitbusBinding::print_error_py)
		.def("print_fatal_error_py", &JitbusBinding::print_fatal_error_py)
		.def("print_set_level_py", &JitbusBinding::print_set_level_py)
        .def("print_enable_color_py", &JitbusBinding::print_enable_color_py)
		.def("available_py", &JitbusBinding::available_py)
		.def("sendPacket_py", &JitbusBinding::sendPacket_py)
		.def("receivePacket_py", &JitbusBinding::receivePacket_py)
		.def("sendPacketBlocking_py", &JitbusBinding::sendPacketBlocking_py)
        .def("sendPacketHz_py", &JitbusBinding::sendPacketHz_py)
		;

}


// struct JitbusBinding_wrap
//   : JitbusBinding, public boost::python::wrapper<JitbusBinding>
// {
//   JitbusBinding_wrap(): JitbusBinding() {};
//   uint8_t read2() { return this->get_override("read2")();  }; 

// };

// BOOST_PYTHON_MODULE(_jitbus_binding_so)
// {
//   namespace python = boost::python;

//   // Expose models.
//   python::class_<JitbusBinding_wrap, boost::noncopyable>(
//       "JitbusBinding", python::init<>())
// //	.def("sendPacket", &JitbusBinding::sendPacketPy)
// //	.def("receivePacket", &JitbusBinding::receivePacketPy)
//     .def("read2", &JitbusBinding_wrap::read2)
//     ;
// }



/*
BOOST_PYTHON_MODULE(_jitbus_binding_so)
{
  boost::python::class_<JitbusBinding>("JitbusBinding", boost::python::init<>())
	.def("sendPacket", &JitbusBinding::sendPacketPy)
	.def("receivePacket", &JitbusBinding::receivePacketPy)
	;
}
*/
#endif