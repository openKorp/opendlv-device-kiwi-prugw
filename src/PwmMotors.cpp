/**
 * Copyright (C) 2018 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <algorithm>
#include <chrono> //milliseconds
#include <thread> //thread sleep
#include <fstream>
#include <sstream>
#include <iostream>
#include <math.h>


#include <fcntl.h> // for open
#include <unistd.h> // for close
#include <sys/mman.h> // mmap
#include <cstring> // for memset

#include <sys/stat.h> // checking if file/dir exist

#include "PwmMotors.h"



PwmMotors::PwmMotors(std::vector<std::string> a_names, 
    std::vector<std::string> a_types,
    std::vector<std::string> a_channels,
    std::vector<std::string> a_offets,
    std::vector<std::string> a_maxvals
  )
    : m_motors()
    , m_mutex()
    , m_prusharedMemInt32_ptr{NULL}
    , m_SERVO_PRU_CH{1} // PRU1
    , m_SERVO_PRU_FW{"am335x-pru1-rc-servo-fw"} //"am335x-pru1-rc-servo-fw"
    , m_PRU0_STATE{"/sys/class/remoteproc/remoteproc1/state"}
    , m_PRU1_STATE{"/sys/class/remoteproc/remoteproc2/state"}
    , m_PRU0_FW{"/sys/class/remoteproc/remoteproc1/firmware"}
    , m_PRU1_FW{"/sys/class/remoteproc/remoteproc2/firmware"}
{

  if (a_names.size() == a_channels.size() && a_names.size() == a_types.size()
      && a_channels.size() <= NUM_SERVO_CHANNELS && a_channels.size() > 0 &&
      a_names.size() ==  a_offets.size() && a_names.size() == a_maxvals.size()) {
    for (uint8_t i = 0; i < a_names.size(); ++i) {
      std::string name = a_names.at(i);
      std::string type  = a_types.at(i);
      uint8_t channel = std::stoi(a_channels.at(i));
      Motor::MotorType motorType;
      std::transform(type.begin(), type.end(), type.begin(), ::toupper);
      float offset = std::stof(a_offets.at(i));
      float maxval = std::stof(a_maxvals.at(i));
      if (type.compare("SERVO") == 0) {
        motorType = Motor::MotorType::Servo;
        m_motors.push_back(Motor(name, motorType, channel, offset, maxval));
      } else if (type.compare("ESC") == 0) {
        motorType = Motor::MotorType::Esc;
        m_motors.push_back(Motor(name, motorType, channel, offset, maxval));
      } else {
        std::cerr << " Incorrect configuration for motor type.\n";
        exit(1);
      }
    }
    initialisePru();
    powerServoRail(true);
  } else {
    std::cerr << " Invalid number of configurations for pwm motors.\n";
  }
}

void PwmMotors::initialisePru()
{
  int32_t fileDescriptor;
  terminatePru();
  std::ofstream fileFirmware(m_PRU1_FW);
  fileFirmware << m_SERVO_PRU_FW;
  fileFirmware.flush();
  fileFirmware.close();
  volatile uint32_t  *pru;   // Points to start of PRU memory.
  struct stat sb;
  
  // reset memory pointer to NULL so if init fails it doesn't point somewhere bad
  m_prusharedMemInt32_ptr = NULL;

  // check if firmware exists
  if (stat(("/lib/firmware/" + m_SERVO_PRU_FW).c_str(), &sb) != 0 || !S_ISREG(sb.st_mode)) {
    std::cerr << " ERROR: missing am335x pru firmware" << std::endl;
  }

  std::ofstream fileState(m_PRU1_STATE);
  fileState << "start";
  fileState.flush();
  fileState.close();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::ifstream fileStatus(m_PRU1_STATE);
  std::string strStatus;
  std::getline(fileStatus, strStatus);
  if(!strStatus.compare("running\n")) {
    std::cerr << " ERROR code: " << strStatus << std::endl;  
  }

  // start mmaping shared memory
  fileDescriptor = open("/dev/mem", O_RDWR | O_SYNC);
  if (fileDescriptor == -1) {
    std::cerr << " ERROR: could not open /dev/mem." << std::endl;
  }
  
  pru = static_cast<volatile uint32_t*>(mmap(0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fileDescriptor, PRU_ADDR));
  if (pru == MAP_FAILED) {
    std::cerr << " ERROR: could not map memory." << std::endl;
  }
  close(fileDescriptor);

  m_prusharedMemInt32_ptr = pru + PRU_SHAREDMEM / 4; // Points to start of shared memory
  if (m_prusharedMemInt32_ptr == NULL) {
    std::cerr << " ERROR: pru shared mem is null." << std::endl;
  }

  std::cout << "Successfully initialized servo/esc PRU" << std::endl;
  // std::memset(m_prusharedMemInt32_ptr, 0, SERVO_CHANNELS * 4);
  for (uint8_t i = 8; i < NUM_SERVO_CHANNELS; i++) {
    m_prusharedMemInt32_ptr[i] = 0;
  }
}

void PwmMotors::terminatePru()
{
  std::ofstream file(m_PRU1_STATE);
  file << "stop";
  file.flush();
  file.close();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

PwmMotors::~PwmMotors() 
{
  setServoNormalizedAll(0);
  terminatePru();
  powerServoRail(false);
  m_prusharedMemInt32_ptr = NULL;
}


void PwmMotors::setMotorPower(uint8_t const &a_ch, float const &a_power) {
  for (uint8_t i = 0; i < m_motors.size(); ++i) {
    if (m_motors.at(i).getChannel() == a_ch) {
      std::lock_guard<std::mutex> l(m_mutex);
      m_motors.at(i).setPower(a_power);
      break;
    }
  }
}

std::string PwmMotors::toString()
{
  std::stringstream ss;
  for (uint8_t i = 0; i < m_motors.size(); ++i) {
    ss << "\t" << m_motors[i].toString() << std::endl;
  }
  return ss.str();
}

void PwmMotors::actuate()
{
  std::lock_guard<std::mutex> l(m_mutex);
  for (uint8_t i = 0; i < m_motors.size(); ++i) {
    switch (m_motors.at(i).getType()) {
      case Motor::MotorType::Esc:
        setEscNormalized(m_motors.at(i).getChannel(), m_motors.at(i).getPower());
        break;
      case Motor::MotorType::Servo:
        setServoNormalized(m_motors.at(i).getChannel(), m_motors.at(i).getPower());
        break;
      default:
        break;
    }
  }
}

void PwmMotors::powerServoRail(bool const &a_val)
{
  if (a_val){
    struct stat sb;
    if (stat("/sys/class/gpio/gpio80", &sb) != 0) {
      write2file("/sys/class/gpio/export", "80");
    }
    write2file("/sys/class/gpio/gpio80/direction", "out");
    write2file("/sys/class/gpio/gpio80/value", "1");
  } else {
    write2file("/sys/class/gpio/gpio80/value", "0");
    write2file("/sys/class/gpio/gpio80/direction", "in");
    write2file("/sys/class/gpio/unexport", "80");
  }
}

void PwmMotors::write2file(std::string const &a_path, std::string const &a_str)
{
  std::ofstream file(a_path, std::ofstream::out);
  if (file.is_open()) {
    file << a_str;
  } else {
    std::cerr << " Could not open " << a_path 
        << "." << std::endl;
  }    
  file.flush();
  file.close();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/*******************************************************************************
* Sends a single pulse of duration us (microseconds) to a channels.
* This must be called regularly (>40hz) to keep servos or ESCs awake.
*******************************************************************************/
int8_t PwmMotors::setPwmMicroSeconds(uint8_t const &a_ch, uint32_t const &a_us)
{
  // Sanity Checks
  if(a_ch < 1 || a_ch > NUM_SERVO_CHANNELS){
    std::cout << " ERROR: Channel must be between 1 and " << NUM_SERVO_CHANNELS << ". \n";
    return -2;
  } if(m_prusharedMemInt32_ptr == NULL){
    std::cout << " ERROR: PRU servo Controller not initialized.\n";
    return -2;
  }

  // first check to make sure no pulse is currently being sent
  if(m_prusharedMemInt32_ptr[a_ch-1] != 0){
    std::cout << " WARNING: Tried to start a new pulse amidst another.\n";
    return -1;
  }

  // PRU runs at 200Mhz. find #loops needed
  uint32_t numLoops = (a_us * 200) / PRU_SERVO_LOOP_INSTRUCTIONS;
  // write to PRU shared memory
  m_prusharedMemInt32_ptr[a_ch-1] = numLoops;
  return 0;
}

/*******************************************************************************
* Sends a single pulse of duration us (microseconds) to all channels.
* This must be called regularly (>40hz) to keep servos or ESCs awake.
*******************************************************************************/
int8_t PwmMotors::setPwmMicroSecondsAll(uint32_t const &a_us)
{
  int ret = 0;
  for (uint8_t i = 1; i <= NUM_SERVO_CHANNELS; i++) {
    int8_t retCh = setPwmMicroSeconds(i, a_us);
    if (retCh == -2) {
      return -2;
    } else if(retCh == -1) {
      ret = -1;
    }
  }
  return ret;
}

int8_t PwmMotors::setServoNormalized(uint8_t const &a_ch, float const &a_input)
{
  if (a_ch < 1 || a_ch > NUM_SERVO_CHANNELS) {
    std::cout << " ERROR: Channel must be between 1 and " << NUM_SERVO_CHANNELS << ". \n";
    return -1;
  }
  if (a_input < -1.5f || a_input > 1.5f) {
    std::cout << " ERROR: Servo normalized input must be between -1.5 and 1.5\n";
    return -1;
  }
  uint32_t us = static_cast<uint32_t>(SERVO_MID_US + (a_input * (SERVO_NORMAL_RANGE / 2)));
  return setPwmMicroSeconds(a_ch, us);
}

int8_t PwmMotors::setServoNormalizedAll(float const &a_input)
{
  int ret = 0;
  for (uint8_t i = 1; i <= NUM_SERVO_CHANNELS; i++) {
    int8_t retCh = setServoNormalized(i, a_input);
    if (retCh == -2) {
      return -2;
    } else if(retCh == -1) {
      ret = -1;
    }
  }
  return ret;
}

/*******************************************************************************
* normalized input of 0-1 corresponds to output pulse from 1000-2000 us
* input is allowed to go down to -0.1 so ESC can be armed below minimum throttle
*******************************************************************************/
int8_t PwmMotors::setEscNormalized(uint8_t const &a_ch, float const &a_input)
{
  if (a_ch < 1 || a_ch > NUM_SERVO_CHANNELS){
    std::cout << " ERROR: Channel must be between 1 and " << NUM_SERVO_CHANNELS << ". \n";
    return -1;
  }
  if (a_input < -0.1f || a_input > 1.0f){
    std::cout << " ERROR: ESC normalized input must be between -0.1 & 1\n";
    return -1;
  }
  uint32_t micros = static_cast<uint32_t>(1000 + (a_input * 1000.0f));
  return setPwmMicroSeconds(a_ch, micros);
}

int8_t PwmMotors::setEscNormalizedAll(float const &a_input)
{
  int ret = 0;
  for (uint8_t i = 1; i <= NUM_SERVO_CHANNELS; i++) {
    int8_t retCh = setEscNormalized(i, a_input);
    if (retCh == -2) {
      return -2;
    } else if(retCh == -1) {
      ret = -1;
    }
  }
  return ret;
}


/*******************************************************************************
* int rc_send_oneshot_pulse_normalized(int ch, float input)
* 
* normalized input of 0-1 corresponds to output pulse from 125-250 us
* input is allowed to go down to -0.1 so ESC can be armed below minimum throttle
*******************************************************************************/
int8_t PwmMotors::setEscOneshotNormalized(uint8_t const &a_ch, float const &a_input)
{
  if (a_ch < 1 || a_ch > NUM_SERVO_CHANNELS) {
    std::cout << " ERROR: Channel must be between 1 and " << NUM_SERVO_CHANNELS << ". \n";
    return -1;
  }
  if(a_input < -0.1f || a_input > 1.0f) {
    std::cout << " ERROR: ESC normalized input must be between -0.1 & 1\n";
    return -1;
  }
  uint32_t micros = static_cast<uint32_t>(125.0f + (a_input*125.0f));
  return setPwmMicroSeconds(a_ch, micros);
}

/*******************************************************************************
* int rc_send_oneshot_pulse_normalized_all(float input)
* 
* 
*******************************************************************************/
int8_t PwmMotors::setEscOneshotNormalizedAll(float const &a_input)
{
  int ret = 0;
  for (uint8_t i = 1; i <= NUM_SERVO_CHANNELS; i++) {
    int8_t retCh = setEscOneshotNormalized(i, a_input);
    if (retCh == -2) {
      return -2;
    } else if(retCh == -1) {
      ret = -1;
    }
  }
  return ret;
}



