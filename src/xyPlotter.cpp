#include "xyPlotter.hpp"

xyPlotter::xyPlotter(std::string device) : char_size_(xyCharSize::CHAR_SIZE_8),
                                           parity_(xyParity::PARITY_NONE),
                                           flow_control_(xyFlowControl::FLOW_CONTROL_NONE),
                                           stop_bits_(xyStopBits::STOP_BITS_1),
                                           line_end_(XY_NEWLINE_CARRIAGERETURN),
                                           relative_movement_(false)
{
  serial_port_ = new SerialPort(device);

  serial_port_device_ = device;
}

xyPlotter::~xyPlotter(void){
  delete serial_port_;
}

int xyPlotter::connect(xyBaudRate baud_rate){
  baud_rate_ = baud_rate;

  try{
    serial_port_->Open(baud_rate_,
                      char_size_,
                      parity_,
                      stop_bits_,
                      flow_control_);
  }
  catch (SerialPort::OpenFailed e){
    std::cerr << "Opening the serial port failed!" << std::endl;
    return XY_ERROR;
  }
  catch (SerialPort::AlreadyOpen){
    std::cerr << "Port is already open" << std::endl;
    return XY_ERROR;
  }

  // Not waiting creates unresponsive behaviour
  wait();

  return XY_SUCCESS;
}

void xyPlotter::send(std::string message){
  //if (!serial_port_.good())
  //  return;

  message += line_end_;

  std::vector<unsigned char> buffer;
  while(message.size()){
    buffer.insert(buffer.begin(),
                  message.back());
    message.pop_back();
  }

  serial_port_->Write(buffer);

  wait();
}

std::string xyPlotter::receive(void){
  unsigned int index = 0;
  std::string message = "";
  unsigned char character;

  while(true){
    try{
      character = serial_port_->ReadByte(XY_TIMEOUT);
    }
    catch (SerialPort::ReadTimeout e)
    {
      std::cerr << "Runtime error: Timeout at SerialPort->ReadByte!" << std::endl;
    }

    message += character;

    if (message.rfind(line_end_) != std::string::npos){
      message.pop_back();
// for CR+NL
      if (line_end_.size() != 1)
        message.pop_back();
      return message;
    }

// FIXXME: Is this needed?
//    usleep (100);
   }
 }

double xyPlotter::waitOnOk(void){
  std::chrono::duration<double, std::milli> elapsed;
  std::string message;

  auto start = std::chrono::high_resolution_clock::now();
  while(1){
    message = receive();

    if (message.compare("OK!") == 0)
      return elapsed.count();

    auto now = std::chrono::high_resolution_clock::now();
    elapsed = now - start;

    if(elapsed.count() >= XY_TIMEOUT_MS)
      throw std::runtime_error("Wait on OK timeout");
  }
}

void xyPlotter::moveAbs(double x,
                        double y){

  std::ostringstream stringStream;
  stringStream << XY_MOVE_LINEAR << " X" << x << " Y" << y;

  send(stringStream.str());
  std::cerr << "Moving to (" << x << "," << y << ") waiting on OK...";

  waitOnOk();

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

void xyPlotter::goHome(void){
  // FIXXXME: uncomment
  //if (!serial_port_.good())
  //  return;

  std::cout << "Going to home position" << std::endl;

  send("G28");

  std::cout << "Waiting on ok..." << std::endl;
  waitOnOk();

  x_ = 0.0;
  y_ = 0.0;
}

void xyPlotter::setFrameRate(double fps){
  std::ostringstream stringStream;
  stringStream << "M1 " << fps;
  send(stringStream.str());

  std::cout << "Set hardware Framerate to " << fps << "fps. Waiting on ok...";

  waitOnOk();
}

void xyPlotter::setFlashTime(double milliseconds){
  std::ostringstream stringStream;
  stringStream << "M2 " << milliseconds;

  send(stringStream.str());

  std::cout << "Set Flashtime to " << milliseconds << "ms. Waiting on ok...";

  waitOnOk();
}

void xyPlotter::setLineEnd(std::string line_end){
  line_end_ = line_end;
}

std::string getNewLine(void){

}

void xyPlotter::wait(void){
  std::this_thread::sleep_for(std::chrono::milliseconds(XY_WAIT_MS));
}
