#include "xyPlotter.hpp"

xyPlotter::xyPlotter() : char_size_(xyCharSize::CHAR_SIZE_8),
                         parity_(xyParity::PARITY_NONE),
                         flow_control_(xyFlowControl::FLOW_CONTROL_NONE),
                         num_of_stop_bits_(1),
                         new_line_("\n"),
                         relative_movement_(false)
{}

xyPlotter::~xyPlotter(){}

int xyPlotter::connect(std::string device,
                       xyBaudRate baud_rate){
  serial_port_device_ = device;
  baud_rate_ = baud_rate;

  serial_port_.Open(serial_port_device_);

  serial_port_.SetBaudRate( baud_rate_);
  serial_port_.SetCharSize(char_size_);
  serial_port_.SetParity(parity_);
  serial_port_.SetNumOfStopBits(num_of_stop_bits_);
  serial_port_.SetFlowControl(flow_control_);

  if (!serial_port_.good()){
    return XY_ERROR;
  }
  return XY_SUCCESS;
}

void xyPlotter::send(std::string message){
  message.append(new_line_);

  serial_port_.write(message.c_str(),
                     message.size());
}

void xyPlotter::receive(char* buffer,
  unsigned int buffer_length){

}

void xyPlotter::moveAbs(double x,
                        double y){
  std::ostringstream stringStream;
  stringStream << "G1 X" << x << " Y" << y;

  send(stringStream.str());

  x_ = x;
  y_ = y;
}

void xyPlotter::moveAbsX(double x){
  std::ostringstream stringStream;
  stringStream << "G1 X" << x;

  send(stringStream.str());

  x_ = x;
}

void xyPlotter::moveAbsY(double y){
  std::ostringstream stringStream;
  stringStream << "G1 Y" << y;

  send(stringStream.str());

  y_ = y;
}

void xyPlotter::moveRel(double x,
                        double y){
  moveAbs(x_ + x,
          y_ + y);
}

void xyPlotter::moveRelX(double x){
  moveAbsX(x_ + x);
}

void xyPlotter::moveRelY(double y){
  moveAbsY(y_ + y);
}

void xyPlotter::goHome(){
  send("G28");

  x_ = 0.0;
  y_ = 0.0;
}

void xyPlotter::setFrameRate(double fps){
  std::ostringstream stringStream;
  stringStream << "M1 " << fps;

  send(stringStream.str());
}

void xyPlotter::setFlashTime(double milliseconds){
  std::ostringstream stringStream;
  stringStream << "M2 " << milliseconds;

  send(stringStream.str());
}

void xyPlotter::setNewLine(std::string new_line){
  if (new_line.compare("\n") == 0 ||
      new_line.compare("\r") == 0 ||
      new_line.compare("\r\n") == 0){
    new_line_ = new_line;
  }
  else{
    std::cerr << "Newline characters must be \\n, \\r, or \\r\\n!" << std::endl;
  }
}

std::string getNewLine(){

}
