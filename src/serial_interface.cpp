#include "serial_interface.hpp"

namespace serial_interface{
        serialPort::serialPort(char character_size,
                               char end_of_line_char){
                // create a boost pointer to the serial port interface.
                port_ = shared_ptr<asio::serial_port>
                        (new asio::serial_port(io_));

                character_size_ = character_size;
                end_of_line_char_ = end_of_line_char;
        }

        serialPort::~serialPort(){
                if (port_){
                        port_->cancel();
                        port_->close();
                        port_.reset();
                }
                io_.stop();
                io_.reset();
        }

        int connect(serialPort port,
                    std::string port_name,
                    unsigned int baud_rate){
                system::error_code error_code;

                port.baud_rate_ = baud_rate;

// Early error detection
                if (port.port_){
                        std::cout << "error: The port is already open!" << std::endl;
                        return SERIAL_FAILURE;
                } // error code for port already open

// Open the actual port
                port.port_->open(port_name.c_str(),
                                 error_code);
                if (error_code) {
                        std::cout << "error: port_->open() failed! port_name=" \
                                  << port_name << "\terror_code: " \
                                  << error_code.message().c_str() << std::endl;
                } // if error_code for opening the port failed

// Set the options
                // FIXXME: stop_bits, parity and flow_control could be edited as well
                port.port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
                port.port_->set_option(boost::asio::serial_port_base::character_size(port.character_size_));

// Initialize the thread

                return SERIAL_SUCCESS;
        } // connect

        int disconnect(serialPort port){
                if (port.port_){
                        port.port_->cancel();
                        port.port_->close();
                        port.port_.reset();
                }
                port.io_.stop();
                port.io_.reset();
        } // disconnect
}
