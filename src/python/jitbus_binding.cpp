#ifdef __linux__

#include <boost/python.hpp>
#include "../core/jitcore.h"

class JitbusBinding: public Jitcore {
  
  public:

	JitbusBinding(){}

	~JitbusBinding(){}


 	void sendPacketPy(std::string data, uint16_t data_id){
		
		sendPacket(data.c_str(), data_id);
	}
/*
	bool receivePacketPy(){


	}
*/
	std::string say(std::string msg){

		std::string message = "Calculator Machine says: " + msg; 
		return message;
	}


};

BOOST_PYTHON_MODULE(_jitbus_binding_so)
{
  boost::python::class_<JitbusBinding>("JitbusBinding", boost::python::init<>())
    .def("say", &JitbusBinding::say)
	.def("sendPacket", &JitbusBinding::sendPacketPy)
    ;
}

#endif