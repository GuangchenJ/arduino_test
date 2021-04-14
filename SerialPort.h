#ifndef ARDUINO_TEST_INCLUDE_SERIALPORT_H_
#define ARDUINO_TEST_INCLUDE_SERIALPORT_H_

#include <fcntl.h>  // 包含文件控件，例如O_RDWR
#include <termios.h>  // 包含POSIX终端控件定义
#include <errno.h> // Error integer 和 strerror() 函数
#include <unistd.h> // write(), read(), close()
#include <sys/file.h> // 排他锁
#include <assert.h>

#include <iostream>
#include <string>
#include <cstring>

#define MAX_ARDUINO_WAIT_TIME 255
#define MAX_DATA_LENGTH 256

class SerialPort {
 private:

  bool connected_;
  struct termios tty_;
  int serial_port_;
  //  HANDLE handler_;
  //  COMSTAT status_;
//  DWORD errors_;
  void SetControlModes(const bool __parity,
                       const bool __one_stop_bit,
                       const int __bit_per_byte,
                       const bool __flow_control,
                       const bool __cread_clocal);
  void SetLocalModes();
  void SetInputModes();
  void SetOutputModes();
  void SetVminAndVTime();
  void SetBaudRate(const int __baud_rate);
 public:
  explicit SerialPort(const std::string __portName, const int __baud_rate);
  ~SerialPort();
  void ShowTermios();
  std::string ReadSerialPort(const unsigned int __buf_size);
  int ReadSerialPort(const char *__buffer, const unsigned int __buf_size);
  bool WriteSerialPort(const std::string __buffer);
  bool IsConnected();
  void CloseSerial();
};

#endif //ARDUINO_TEST_INCLUDE_SERIALPORT_H_
