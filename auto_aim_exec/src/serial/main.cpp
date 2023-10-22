#include "serial/serial.hpp"
#include "iostream"
int main(){
  serial::Serial vision_serial("/dev/vision_serial", 115200, serial::Timeout(), 
      serial::bytesize_t(8), serial::parity_t(0), serial::stopbits_t(1), 
      serial::flowcontrol_t(0));
  std::cout << "hello world" << std::endl;
  return 0;
}