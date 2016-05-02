#pragma once
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

#include <string>
#include <iostream>

#define SERIAL_SUCCESS 0
#define SERIAL_FAILURE -1

using namespace boost;

class SerialPort{

protected:
        asio::io_service io_service_;
        shared_ptr<asio::serial_port> port_;

public:
        unsigned int baud_rate_;
        unsigned char character_size_;
        char end_of_line_char_;

private:
        SerialPort(const SerialPort &p);
        SerialPort &operator=(const SerialPort &p);

public:
        SerialPort(void);
        virtual ~SerialPort(void);

        int connect(std::string port_name,
                    unsigned int baud_rate);

        int connect(std::string port_name);

        int disconnect(void);

        int write(const std::string &message);

        int write(const char *message,
                  const int &size);

        int read(void);

};
