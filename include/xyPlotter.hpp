#pragma once

#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <unistd.h>

#include <SerialPort.h>

#include <chrono>
#include <thread>
#include <stdexcept>

#define XY_MOVE_LINEAR "G1"
#define XY_GO_HOME "G28"

#define XY_NEWLINE                "\n"
#define XY_CARRIAGE_RETURN        "\r"
#define XY_NEWLINE_CARRIAGERETURN "\r\n"

#define XY_TIMEOUT 10000
const double XY_TIMEOUT_MS = 20000.0;

#define XY_SUCCESS  0
#define XY_ERROR  -1

#define XY_BUFFER_SIZE  512

#define XY_WAIT_MS  100

typedef SerialPort::BaudRate      xyBaudRate;
typedef SerialPort::CharacterSize xyCharSize;
typedef SerialPort::Parity        xyParity;
typedef SerialPort::FlowControl   xyFlowControl;
typedef SerialPort::StopBits      xyStopBits;

class xyPlotter{
private:
  std::string serial_port_device_;

  xyBaudRate baud_rate_;
  xyCharSize char_size_;
  xyParity parity_;
  xyFlowControl flow_control_;
  xyStopBits stop_bits_;

  std::string line_end_;

  double x_,y_; // Position
  bool relative_movement_;


public:
  SerialPort* serial_port_;

  xyPlotter(std::string device);

  ~xyPlotter(void);

  int connect(xyBaudRate baud_rate);

  void send(std::string message);

  std::string receive(void);

  double waitOnOk(void);

  void moveAbs(double x,
               double y);

  void moveAbsX(double x);

  void moveAbsY(double y);

  void moveRel(double x,
               double y);

  void moveRelX(double x);

  void moveRelY(double y);

  void goHome(void);

  void getPosition(void);

  void setFrameRate(double fps);

  void setFlashTime(double milliseconds);

  void setLineEnd(std::string line_end);

  std::string getNewLine(void);

  void wait(void);
};
