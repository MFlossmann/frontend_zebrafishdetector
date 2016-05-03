#include "xyPlotter.hpp"

xyPlotter::xyPlotter() : char_size_(xyCharSize::CHAR_SIZE_8),
                         parity_(xyParity::PARITY_NONE),
                         flow_control_(xyFlowControl::FLOW_CONTROL_NONE),
                         num_of_stop_bits_(1),
                         line_end_(XY_NEWLINE_CARRIAGERETURN),
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
  message += line_end_;

  serial_port_.write(message.c_str(),
                     message.size());
}

std::string xyPlotter::receive(){
  // FIXXME: This will run forever, if nothing is received
  unsigned int index = 0;
  std::string message = "";
  char character;

// FIXXME: Include a timeout (if possible)
  while(true){
    serial_port_.get(character);
    message += character;

    if (message.rfind(line_end_) != std::string::npos){
      message.pop_back();
// for CR+NL
      if (line_end_.size() != 1)
        message.pop_back();
      return message;
    }

    usleep (100);
  }
}

int xyPlotter::waitOnOk(){
  char buffer[XY_BUFFER_SIZE];
  std::string message;
  std::string ok_message = "OK!";

// Here, the distance should be received.
  message = receive();

  if (ok_message.compare(message) == 0)
    return XY_SUCCESS;
  else{
    return XY_ERROR;
  }
}

void xyPlotter::moveAbs(double x,
                        double y){
  std::ostringstream stringStream;
  stringStream << XY_MOVE_LINEAR << " X" << x << " Y" << y;

  send(stringStream.str());
  std::cout << "Moving, waiting on OK...";

  usleep(1000000);

// FIXXME: Here, a timeout option can be implemented, when the distance is received.
// FIXXXXME: Bad hack, timeout should be included, otherwise, infinite loop might occur.
  while(waitOnOk() != XY_SUCCESS);

  std::cout << "OK! received!" << std::endl;

  x_ = x;
  y_ = y;
}

void xyPlotter::moveAbsX(double x){
  moveAbs(x,y_);
}

void xyPlotter::moveAbsY(double y){
  moveAbs(x_,y);
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
  std::cout << "Going to home position" << std::endl;

  send(XY_GO_HOME);
  waitOnOk();

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

void xyPlotter::setLineEnd(std::string line_end){
  line_end_ = line_end;
}

std::string getNewLine(){

}
