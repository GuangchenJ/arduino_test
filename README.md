# Linux下连接Arduino，并收发信息

Arduino的参考代码在 [`arduino_code`](\arduino_code)中，其中[`DH11_receive_data.ino`](\arduino_code/DH11_receive_data.ino)是使用DH11传感器接受温度信息的代码；

![DH11_receive_data_example](/imgs/DH11_receive_data_example.png)

[`send_data.ino`](/arduino_code/send_data.ino)是收发信息的代码，该代码会将接收到的一切字符串回传显示。

![send_data_example](/imgs/send_data_example.png)

## 类接口

类没有完全写完，具体常用接口有：

```objectivec
-  std::string ReadSerialPort(const unsigned int __buf_size);
-  bool WriteSerialPort(const std::string __buffer);
```

其中函数使用如下表所示

| 函数名            | 函数作用                                    | 参数                                                                                                                       |
| ----------------- | ------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| SerialPort()      | 构造函数                                    | `__portName`：端口名称，与Arduino IDE上 `工具->端口`中显示的一致；`__baud_rate`：波特率，注意与在Arduino中设置的波特率一致 |
| ~SerialPort()     | 析构函数                                    |                                                                                                                            |
| ReadSerialPort()  | 读取Arduino传入的数据（常用只有一个参数的） | `__buf_size`：最大读入字符串大小                                                                                           |
| WriteSerialPort() | 往Arduino传入数据                           | `__buffer`：写入的数据                                                                                                       |
| IsConnected()     | 检测连接是否打开                            |                                                                                                                            |
| CloseSerial()     | 关闭连接                                    |                                                                                                                            |

注意使用时填写好端口`__portName`和波特率`__baud_rate`，否则会出现BUG。还有注意好睡眠时间要和Arduino中的`delay()`一致。

![cpp_receive_data_example](/imgs/cpp_receive_data_example.png)

有时数据接受会有缺失，这个需要处理（还没处理）

![cpp_send_data_example](/imgs/cpp_send_data_example.png)

传入数据程序是一直向其传入"Hello World!"，然后Arduino收到后返回给C++程序。
