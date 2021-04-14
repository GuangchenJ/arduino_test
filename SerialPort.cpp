#include "SerialPort.h"

void SerialPort::ShowTermios() {
  std::cout << "input mode flags: " << this->tty_.c_iflag << std::endl
            << "output mode flags: " << this->tty_.c_oflag << std::endl
            << "control mode flags: " << this->tty_.c_cflag << std::endl
            << "local mode flags: " << this->tty_.c_lflag << std::endl
            << "line discipline: " << this->tty_.c_lflag << std::endl
            << "input speed: " << this->tty_.c_cc << std::endl
            << "output speed: " << this->tty_.c_cc << std::endl;
}

void SerialPort::SetControlModes(const bool __parity = false,
                                 const bool __one_stop_bit = false,
                                 const int __bit_per_byte = 8,
                                 const bool __flow_control = false,
                                 const bool __cread_clocal = true) {

  // set parity
  // 如果该位置为1，就启用奇偶校验位的生成与检测，这里不使用
  if (__parity) {
    this->tty_.c_cflag |= PARENB;;  // 设置奇偶校验位，启用奇偶校验
  } else {
    this->tty_.c_cflag &= ~PARENB;  // 清除奇偶校验位，禁用奇偶校验（最常见）
  }

  // 该位置为1就使用两个停止位，清除则为一个停止位，大多数为一个停止位
  if (__one_stop_bit) {
    this->tty_.c_cflag |= CSTOPB;  // 设置停止字段，通信中使用两个停止位
  } else {
    this->tty_.c_cflag &= ~CSTOPB; // 清除停止字段，仅在通信中使用一个停止位（最常见）
  }

  // 设置每个字节多少位
  this->tty_.c_cflag &= ~CSIZE; // 清除所有大小位
  switch (__bit_per_byte) {
    case 5: {
      this->tty_.c_cflag |= CS5; // 5 bits per byte
      break;
    }
    case 6: {
      this->tty_.c_cflag |= CS6; // 6 bits per byte
      break;
    }
    case 7: {
      this->tty_.c_cflag |= CS7; // 7 bits per byte
      break;
    }
    case 8: {
      this->tty_.c_cflag |= CS8; // 8 bits per byte (most common)
      break;
    }
    default: {
      this->tty_.c_cflag |= CSIZE; // 8 bits per byte (most common)
    }
  }

  // 流量控制，启用可能导致不会收到任何数据
  if (__flow_control) {
    this->tty_.c_cflag |= CRTSCTS;  // 启用RTS/CTS硬件流控制
  } else {
    this->tty_.c_cflag &= ~CRTSCTS; // 禁用RTS/CTS硬件流控制（最常见）
  }


  // 设置CLOCAL将禁用调制解调器特定的信号线，例如载波检测。
  // 当检测到调制解调器断开连接时，还可以使用SIGHUP防止控制过程发送信号，通常设置较好。设置CLOCAL使我们能够读取数据
  if (__cread_clocal)
    this->tty_.c_cflag |= CREAD | CLOCAL;
}

void SerialPort::SetLocalModes() {
  // 禁用规范模式，不禁用可能会导致字节丢失
  this->tty_.c_lflag &= ~ICANON;

  // Echo，如果设1，发送的字符将会被回显，以防万一便禁用
  this->tty_.c_lflag &= ~ECHO; // 禁用 echo
  this->tty_.c_lflag &= ~ECHOE; // 禁用 erasure
  this->tty_.c_lflag &= ~ECHONL; // 禁用换行 echo

  // 禁用信号字符，不使用串行端口，清除以下位
  this->tty_.c_lflag &= ~ISIG; // 禁止解释INTR，QUIT和SUSP
}

void SerialPort::SetInputModes() {
  // 禁用软件流控制
  this->tty_.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

  // 禁用接收时字节的特殊处理
  this->tty_.c_iflag &=
      ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

}

void SerialPort::SetOutputModes() {
  // 在配置串行端口时，我们要禁用对输出字符/字节的任何特殊处理
  this->tty_.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  this->tty_.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  //this->tty_.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
  //this->tty_.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
}

// VMIN = 0时，VTIME代表从第一次read()调用开始后经过的时间
// VMIN > 0时，VTIME代表第一次读入字符后经过的时间
// VMIN = 0，VTIME = 0：无阻塞，立即返回可用值
// VMIN > 0，VTIME = 0：这将read()始终等待字节的传入等待（VMIN确定），因此read()可以无限期地阻塞。
// VMIN = 0，VTIME > 0：这是最大超时时间（VTIME确定）的任何数字字符的阻塞读取。read()将阻塞到有大量数据可用或发生超时为止。这恰好是我最喜欢的模式（也是我最常使用的模式）。
// VMIN > 0，VTIME > 0：阻塞直到VMIN接收到任何字符或VTIME第一个字符过去。请注意，VTIME直到收到第一个字符，超时才会开始。
// VMIN和VTIME两者都定义为type cc_t，我一直看到它是unsigned char（1个字节）的别名。这将VMIN字符数的上限设置为255，最大超时为25.5秒（255分秒）。
void SerialPort::SetVminAndVTime() {
  // Wait for up to 0.1*MAX_ARDUINO_WAIT_TIME s (MAX_ARDUINO_WAIT_TIME deciseconds), returning as soon as any data is received.
  this->tty_.c_cc[VTIME] = MAX_ARDUINO_WAIT_TIME;
  this->tty_.c_cc[VMIN] = 0;
}

void SerialPort::SetBaudRate(const int __baud_rate = 9600) {
  // 设置读写速率
  switch (__baud_rate) {
    case 0: {
      cfsetspeed(&this->tty_, B0);
      break;
    }
    case 50: {
      cfsetspeed(&this->tty_, B50);
      break;
    }
    case 75: {
      cfsetspeed(&this->tty_, B75);
      break;
    }
    case 110: {
      cfsetspeed(&this->tty_, B110);
      break;
    }
    case 134: {
      cfsetspeed(&this->tty_, B134);
      break;
    }
    case 150: {
      cfsetspeed(&this->tty_, B150);
      break;
    }
    case 200: {
      cfsetspeed(&this->tty_, B200);
      break;
    }
    case 300: {
      cfsetspeed(&this->tty_, B300);
      break;
    }
    case 600: {
      cfsetspeed(&this->tty_, B600);
      break;
    }
    case 1200: {
      cfsetspeed(&this->tty_, B1200);
      break;
    }
    case 1800: {
      cfsetspeed(&this->tty_, B1800);
      break;
    }
    case 2400: {
      cfsetspeed(&this->tty_, B2400);
      break;
    }
    case 4800: {
      cfsetspeed(&this->tty_, B4800);
      break;
    }
    case 9600: {
      cfsetspeed(&this->tty_, B9600);
      break;
    }
    case 19200: {
      cfsetspeed(&this->tty_, B19200);
      break;
    }
    case 38400: {
      cfsetspeed(&this->tty_, B38400);
      break;
    }
    case 57600: {
      cfsetspeed(&this->tty_, B57600);
      break;
    }
    case 115200: {
      cfsetspeed(&this->tty_, B115200);
      break;
    }
    case 230400: {
      cfsetspeed(&this->tty_, B230400);
      break;
    }
    case 460800: {
      cfsetspeed(&this->tty_, B460800);
      break;
    }
    default: {
      assert(false);
    }
  }
}

SerialPort::SerialPort(const std::string __port_name, const int __baud_rate) {
  this->connected_ = false;

  // 尝试打开串口
  this->serial_port_ = open(__port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (-1 == this->serial_port_) {
    std::cerr << "Open_port: Unable to open " + __port_name << std::endl;
  } else {
    fcntl(this->serial_port_, F_SETFL, 0);
    std::cout << "Test Port " + __port_name + " has been successfully opened and " << this->serial_port_
              << " is the file description" << std::endl;

    // 获取当前的串口设置
    if (tcgetattr(this->serial_port_, &this->tty_)) {
      std::cerr << "Error " << errno << " from tcgetattr: " << std::strerror(errno) << std::endl;
    }

    this->SetControlModes();
    this->SetLocalModes();
    this->SetInputModes();
    this->SetOutputModes();
    this->SetBaudRate(__baud_rate);

    this->ShowTermios();

    // Save tty settings, also checking for error
    if (tcsetattr(this->serial_port_, TCSANOW, &this->tty_)) {
      std::cerr << "Error " << errno << "from tcsetattr: " << strerror(errno) << std::endl;
    }

    this->connected_ = true;

    // 获取非阻塞排他锁
    // EACCES：访问出错
    // EAGAIN：文件已被锁定，或者太多的内存已被锁定
    // EBADF：fd不是有效的文件描述词
    // EINVAL：一个或者多个参数无效
    // ENFILE：已达到系统对打开文件的限制
    // ENODEV：指定文件所在的文件系统不支持内存映射
    // ENOMEM：内存不足，或者进程已超出最大内存映射数量
    // EPERM：权能不足，操作不允许
    // ETXTBSY：已写的方式打开文件，同时指定MAP_DENYWRITE标志
    // SIGSEGV：试着向只读区写入
    // SIGBUS：试着访问不属于进程的内存区
    if (flock(this->serial_port_, LOCK_EX | LOCK_NB)) {
      throw std::runtime_error("Serial port with file descriptor " +
          std::to_string(this->serial_port_) + " is already locked by another process.");
    }
  }
}

SerialPort::~SerialPort() {
  if (this->connected_) {
    this->connected_ = false;
    close(this->serial_port_);
  }
}

// Reading bytes from serial port to buffer;
// returns read bytes count, or if error occurs, returns -1
// 函数有点问题，一般不使用这个函数
int SerialPort::ReadSerialPort(const char *__buffer, const unsigned int __buf_size) {
  // todo: 更改__buffer值
  // Allocate memory for read buffer, set size according to your needs
  unsigned int buf_size;
  if (__buf_size > MAX_DATA_LENGTH)
    buf_size = MAX_DATA_LENGTH;
  else
    buf_size = __buf_size;
  char read_buf[buf_size];

  // 通常不会执行这个memset()调用，但不加会有乱码
  memset(&read_buf, '\0', sizeof(read_buf));

  // 读取字节。read()的行为取决于上面的配置设置，特别是VMIN和VTIME
  int num_bytes = read(this->serial_port_, &read_buf, sizeof(read_buf));

  // n is the number of bytes read.
  // n may be 0 if no bytes were received,
  // and can also be -1 to signal an error.
  if (num_bytes < 0) {
    std::cerr << "Error reading: " << strerror(errno) << std::endl;
    return -1;
  }
  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
  // print it to the screen like this!)
  std::cout << "Read " << num_bytes << " bytes. Received message: " << read_buf << std::endl;
  return sizeof(read_buf);
}

// Reading bytes from serial port to buffer;
// 返回读取的字符串（无错误处理）
std::string SerialPort::ReadSerialPort(const unsigned int __buf_size) {
  // Allocate memory for read buffer, set size according to your needs
  unsigned int buf_size;
  if (__buf_size > MAX_DATA_LENGTH)
    buf_size = MAX_DATA_LENGTH;
  else
    buf_size = __buf_size;
  char read_buf[buf_size];

  // 通常不会执行这个memset()调用，但不加会有乱码
  memset(&read_buf, '\0', sizeof(read_buf));

  // 读取字节。read()的行为取决于上面的配置设置，特别是VMIN和VTIME
  int num_bytes = read(this->serial_port_, &read_buf, sizeof(read_buf));

  // n is the number of bytes read.
  // n may be 0 if no bytes were received,
  // and can also be -1 to signal an error.
  if (num_bytes < 0) {
    std::cerr << "Error reading: " << strerror(errno) << std::endl;
    return std::string(read_buf);
  }
  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
  // print it to the screen like this!)
  return std::string(read_buf);
}

bool SerialPort::WriteSerialPort(const std::string __buffer) {
  write(this->serial_port_, __buffer.c_str(), __buffer.size());
}

bool SerialPort::IsConnected() {
  // todo: 完善判断条件
  return this->connected_;
}

void SerialPort::CloseSerial() {
  if (this->connected_) {
    this->connected_ = false;
    close(this->serial_port_);
  }
}