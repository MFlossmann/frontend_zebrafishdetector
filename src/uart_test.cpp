#include <SerialStream.h>
#include <iostream>
#include <math.h>
#include <sstream>
#include <unistd.h>

main(int argc, char* argv[]){
  const char* const SERIAL_PORT_DEVICE = "/dev/ttyUSB0";

  LibSerial::SerialStream serial_port;

  serial_port.Open( SERIAL_PORT_DEVICE );
  if (!serial_port.good()) {
    std::cerr << "Error: Could not open serial port " << std::endl;
    exit(1);

  }

// Set the baud rate
  serial_port.SetBaudRate( LibSerial::SerialStreamBuf::BAUD_57600);
  if (!serial_port.good()) {
    std::cerr << "Error: Could not set the baud rate!" << std::endl;
    exit(1);
  }

// Set number of data bits.
  serial_port.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
  if (!serial_port.good()) {
    std::cerr << "Error: Could not set the char size!" << std::endl;
    exit(1);
  }

// Disable parity.
  serial_port.SetParity(LibSerial::SerialStreamBuf::PARITY_NONE);
  if (!serial_port.good()) {
    std::cerr << "Error: Could not set the parity!" << std::endl;
    exit(1);
  }

// Set number of stop bits.
  serial_port.SetNumOfStopBits(1);
  if (!serial_port.good()) {
    std::cerr << "Error: Could not set the number of stop bits!" << std::endl;
    exit(1);
  }

// Turn on hardware flow control.
  serial_port.SetFlowControl(LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE);
  if (!serial_port.good()) {
    std::cerr << "Error: Could not set the flow control!" << std::endl;
    exit(1);
  }

  std::cout << "Sending the stuff..." << std::endl;
  std::cout << "narf" << std::endl;
  for (int i=1; i<=8;i++){
    double x = 10*pow(-1.0,i % 3);
    std::ostringstream stringStream;
    stringStream << "G1 X" << x << '\n';
    std::string message = stringStream.str();
    serial_port.write(message.c_str(),
                      message.size());
    std::cout << message << "..." << std::endl;
    usleep(4000000);
  }

  return 0;
}
