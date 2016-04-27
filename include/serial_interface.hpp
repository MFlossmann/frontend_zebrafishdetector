#pragma once
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

#include <string>

#define SERIAL_SUCCESS 0
#define SERIAL_FAILURE -1

using namespace boost;

namespace serial_interface{
        struct serialPort{
                serialPort(char character_size = 8,
                           char end_of_line_char = '\n');
                virtual ~serialPort();

                asio::io_service io_;
                shared_ptr<asio::serial_port> port_;

                unsigned int baud_rate_;
                unsigned char character_size_;
                char end_of_line_char_;
        };

        int connect(serialPort port);

        int disconnect(serialPort port);

        int write(serialPort port,
                  const std::string &buffer);

        int write(serialPort port,
                  const char *buffer,
                  const int &size);

        int read(serialPort port);
}
