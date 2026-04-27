#include <iostream>

#define JITBUS_DISABLE_LOG
#include <jitbus.h>

SerialJitbus jit;

struct Counter {
  uint32_t seq;
};

Counter counter_sender; // Counter struct message, has to be same on the receiver
int counter_sender_id = 57; // Counter identificator, has to be same both on the receiver
uint32_t counter_timer = 0; // Auxiliar variable required for sendPacketHz()

Counter counter_receiver; // Counter struct message, has to be same on the sender
int counter_receiver_id = 57; // Counter identificator, has to be same on the sender

int main(void)
{
  counter_sender.seq = 0;
  counter_receiver.seq = 0;

  if (jit.init("/dev/ttyUSB0", 115200))
  {
    std::cout << "Serial port opened! \n";
  }

  while(true)
  {

    if (jit.sendPacketHz(counter_sender, counter_sender_id, counter_timer, 2)){
        std::cout << "Sent: Counter.seq = " << counter_sender.seq << "\n";
        counter_sender.seq++;
    };

    if (jit.available() > 0)
    {
      if (jit.receivePacket(counter_receiver, counter_receiver_id))
      {
        std::cout << "Received: Counter.seq = " << counter_receiver.seq << "\n";
      }
    }
  }

  return 0;
}