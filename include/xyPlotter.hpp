#pragma once

#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <unistd.h>

#include <SerialStream.h>

#define XY_MOVE_LINEAR "G1"
#define XY_GO_HOME "G28"

#define XY_NEWLINE                "\n"
#define XY_CARRIAGE_RETURN        "\r"
#define XY_NEWLINE_CARRIAGERETURN "\r\n"

#define XY_SUCCESS  0
#define XY_ERROR  -1

#define XY_BUFFER_SIZE  512

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

  std::string line_end_;

  double x_,y_; // Position
  bool relative_movement_;


public:
  SerialStream serial_port_;

  xyPlotter();

  ~xyPlotter();

  int connect(std::string device,
              xyBaudRate baud_rate);

  void send(std::string message);

  std::string receive();

  int waitOnOk();

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

  void setLineEnd(std::string line_end);

  std::string getNewLine();
};
