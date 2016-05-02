#pragma once

#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <SerialStream.h>

#define XY_SUCCESS  0
#define XY_ERROR  -1

using namespace LibSerial;

typedef SerialStreamBuf::BaudRateEnum      xyBaudRate;
typedef SerialStreamBuf::CharSizeEnum      xyCharSize;
typedef SerialStreamBuf::ParityEnum        xyParity;
typedef SerialStreamBuf::FlowControlEnum   xyFlowControl;

class xyPlotter{
private:
  std::string serial_port_device_;

  xyBaudRate baud_rate_;
  xyCharSize char_size_;
  xyParity parity_;
  xyFlowControl flow_control_;

  char num_of_stop_bits_;

  std::string new_line_;

  double x_,y_; // Position
  bool relative_movement_;


public:
  SerialStream serial_port_;

  xyPlotter();

  ~xyPlotter();

  int connect(std::string device,
              xyBaudRate baud_rate);

  void send(std::string message);

  void receive(char* buffer,
               unsigned int buffer_length);

  void moveAbs(double x,
               double y);

  void moveAbsX(double x);

  void moveAbsY(double y);

  void moveRel(double x,
               double y);

  void moveRelX(double x);

  void moveRelY(double y);

  void goHome();

  void getPosition();

  void setFrameRate(double fps);

  void setFlashTime(double milliseconds);

  void setNewLine(std::string new_line);

  std::string getNewLine();
};
