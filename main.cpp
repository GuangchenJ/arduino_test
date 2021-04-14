#include <iostream>
#include "SerialPort.h"

int main() {
  SerialPort serial_port("/dev/ttyUSB0", 9600);
  for (int i = 0; i < 1000; ++i) {
    serial_port.WriteSerialPort("Hello World! " + std::to_string(i + 1));
    std::cout << serial_port.ReadSerialPort(MAX_DATA_LENGTH) << std::endl;
    sleep(2);
  }
  return 0;
}