#include <fcntl.h>   // 文件控制定义
#include <termios.h> // 终端I/O函数定义
#include <unistd.h>  // UNIX标准函数定义
#include <time.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <mutex>
#include <thread>
#include "data.hpp"
using namespace std;
/*

定义状态:

SEARCHING: 搜索帧头0xFC状态
FOUND_HEADER: 找到帧头状态
GET_LENGTH: 获取数据长度状态
IN_FRAME: 在处理帧数据状态

*/
enum State
{
    SEARCHING,
    FOUND_HEADER,
    IN_FRAME
};
enum State_Write
{
    CONTENT,
    HEADER_LEN_TYPE,
    SEND
};

class uart
{
private:
    string name = "/dev/ttyUSB0";
    int fd;
    int open_mode = O_RDWR | O_NOCTTY | O_NDELAY;
    int baudrate = 115200;
    int databits = CS8;
    int stopbits = 0;
    int polarity = 0;
    int flow_ctrl = 0;
    int length = 0, type = 0;
    uint8_t length_flag = 0, length_target = 0;
    uint8_t sum = 0;
    int target_found = 0;
    uint8_t read_buf[MAX_LEN] = {0};
    uint8_t write_buf[MAX_LEN] = {0};
    State state = SEARCHING;
    State_Write state_write = CONTENT;
    // mutex mtx_uart;

public:
    uart(){};
    uart(string uart_name, int open_mode, int baudrate, int databits, int stopbits, int polality, int flow_ctrl);
    ~uart();
    void uart_init()
    {
        // lock_guard<mutex> lock(mtx_uart);

        // 打开串口设备文件,指定只读写模式,不控制串口
        int fd = open(this->name.c_str(), this->open_mode);
        if (fd == -1)
        {
            throw std::runtime_error("USART: Can't open " + this->name + "\n");
        }
        /*测试是否为终端设备*/
        if (isatty(fd) == 0)
            throw std::runtime_error("USART: Standard input is not a terminal device\n");
        this->fd = fd;
        // 设置文件描述符fd的异步I/O属性
        struct serial_struct serial;
        ioctl(this->fd, TIOCGSERIAL, &serial);
        // 设置低延迟模式
        serial.flags |= ASYNC_LOW_LATENCY;
        ioctl(this->fd, TIOCSSERIAL, &serial);
        struct termios newtio;
        uart_struct_init(&newtio);
        config_uart_struct(newtio);
    }
    void uart_struct_init(struct termios *newtio)
    {
        struct termios oldtio;
        /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/
        if (tcgetattr(this->fd, &oldtio) != 0)
            throw std::runtime_error("USART: tcgetattr( fd,&oldtio) -> %d");

        /*步骤一，设置字符大小-八位数据*/
        newtio->c_cflag |= CLOCAL | CREAD;
        newtio->c_cflag &= ~CSIZE;
        newtio->c_cflag |= CS8;
        /*设置奇偶校验位-不校验*/
        newtio->c_cflag &= ~PARENB;
        newtio->c_iflag &= ~INPCK;
        newtio->c_iflag = 0;
        /*设置波特率115200*/
        cfsetispeed(newtio, B115200);
        cfsetospeed(newtio, B115200);
        /*设置停止位*/
        newtio->c_cflag &= ~CSTOPB;
        newtio->c_cflag |= IXON | IXOFF | IXANY;

        /*设置等待时间和最小接收字符*/
        newtio->c_cc[VTIME] = 0;
        newtio->c_cc[VMIN] = 1;
    }
    void config_uart_struct(struct termios &tio)
    {
        /*处理未接收字符*/
        tcflush(this->fd, TCIFLUSH);
        /*激活新配置*/
        if ((tcsetattr(this->fd, TCSANOW, &tio)) != 0)
            throw std::runtime_error("USART: tcsetattr error!");
    }
    void my_read(my_data &data)
    {
        // lock_guard<mutex> lock(mtx_uart);
        while (true)
        {

            if (this->state == SEARCHING)
            {
                read(fd, this->read_buf, 1);
                if (this->read_buf[0] == 0xFC)
                {
                    this->state = FOUND_HEADER;
                }
            }
            if (this->state == FOUND_HEADER)
            {
                read(fd, this->read_buf, 1);
                if (this->read_buf[0] == 0xFC)
                {
                    this->state = SEARCHING;
                }
                else
                {
                    length = this->read_buf[0] & 0x7f;
                    type = this->read_buf[0] >> 7;
                    // check the length
                    if (length > 20)
                    {
                        this->state = SEARCHING;
                        break;
                    }
                    // printf("%x\t%x\t%x\n", this->read_buf[0], length, type);
                    this->state = IN_FRAME;
                }
            }
            if (this->state == IN_FRAME)
            {
                read(this->fd, this->read_buf, length);
                // 将帧数据添加到待处理队列中，数据会在其他线程处理
                SerialData frame;
                frame.len = this->length;
                frame.pack_type = this->type;
                memcpy(frame.storage, this->read_buf, this->length);
                my_time mt;
                frame.timestamp = mt;
                data.ReceiveQueue.push_back(frame);
                length = 0;
                this->state = SEARCHING;
                break;
            }
        }
    }
    void my_write(my_data &data)
    {
        // lock_guard<mutex> lock(mtx_uart);

        SerialData pack = data.SendQueue.front();
        data.SendQueue.pop_front();
        while (true)
        {
            if (this->state_write == CONTENT)
            {
                if (pack.pack_type == 2)
                {
                    // only flags
                    for (int i = 0; i < 7; i++)
                    {
                        if (pack.storage[i] == 0xFC)
                        {
                            this->write_buf[2 + this->length_flag] = 0xFC;
                            this->length_flag++;
                        }
                        this->write_buf[2 + this->length_flag] = pack.storage[i];
                        this->length_flag++;
                    }
                }
                else if (pack.pack_type == 3)
                {
                    // flags & pose
                    for (int i = 0; i < 7; i++)
                    {
                        if (pack.storage[i] == 0xFC)
                        {
                            this->write_buf[2 + this->length_flag] = 0xFC;
                            this->length_flag++;
                        }
                        this->write_buf[2 + this->length_flag] = pack.storage[i];
                        this->length_flag++;
                    }
                    for (int i = 7; i < 15; i++)
                    {
                        if (pack.storage[i] == 0xFC)
                        {
                            this->write_buf[4 + this->length_flag + this->length_target] = 0xFC;
                            this->length_target++;
                        }
                        this->write_buf[4 + this->length_target + this->length_flag] = pack.storage[i];
                        this->length_target++;
                    }
                }
                // 遇到0xFC重复一遍
                this->state_write = HEADER_LEN_TYPE;
            }
            if (this->state_write == HEADER_LEN_TYPE)
            {
                if (pack.pack_type == 2)
                {
                    // only flags
                    this->write_buf[0] = 0xFC;
                    this->write_buf[1] = this->length_flag & 0xff;
                }
                else if (pack.pack_type == 3)
                {
                    // flags & pose
                    this->write_buf[0] = 0xFC;
                    this->write_buf[1] = this->length_flag & 0xff;
                    this->write_buf[this->length_flag + 2] = 0xFC;
                    this->write_buf[this->length_flag + 3] = this->length_target | 0x80 & 0xff;
                }
                this->state_write = SEND;
            }
            if (this->state_write == SEND)
            {
                if (pack.pack_type == 2)
                {
                    write(this->fd, this->write_buf, this->length_flag + 2);
                    for (int i = 0; i < this->length_flag + this->length_target + 2; i++)
                    {
                        // printf("%x ", this->write_buf[i]);
                        this->write_buf[i] = 0x00;
                    }
                    // printf("\n");
                }
                else if (pack.pack_type == 3)
                {
                    write(this->fd, this->write_buf, this->length_flag + this->length_target + 4);
                    for (int i = 0; i < this->length_flag + this->length_target + 4; i++)
                    {
                        // printf("%x ", this->write_buf[i]);
                        this->write_buf[i] = 0x00;
                    }
                    // printf("\n");
                }
                this->length_flag = 0;
                this->length_target = 0;
                this->state_write = CONTENT;
                break;
            }
        }
    }
};

uart::uart(string uart_name, int open_mode, int baudrate, int databits, int stopbits, int polality, int flow_ctrl)
{
    this->name = uart_name;
    this->open_mode = open_mode;
    this->baudrate = baudrate;
    this->databits = databits;
    this->stopbits = stopbits;
    this->polarity = polality;
    this->flow_ctrl = flow_ctrl;
}

uart::~uart()
{
}