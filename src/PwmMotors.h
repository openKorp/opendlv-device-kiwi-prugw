/**
 * Copyright (C) 2018 Björnborg Ngúyen
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


#include <memory>
#include <vector>
#include <mutex>



#include "Motor.h"

/**
 * Interface to motor through pwm signals.
 */
class PwmMotors {
 public:
  PwmMotors(std::vector<std::string>, std::vector<std::string>, std::vector<std::string>, std::vector<std::string>, std::vector<std::string>);
  virtual ~PwmMotors();
  void actuate();
  std::string toString();
  void setMotorPower(uint8_t, float);
  void initialisePru();
  void powerServoRail(bool);

 private:
  PwmMotors(const PwmMotors &) = delete;
  PwmMotors(PwmMotors &&) = delete;
  PwmMotors &operator=(const PwmMotors &) = delete;
  PwmMotors &operator=(PwmMotors &&) = delete;

  int32_t getPruEncoderPos();
  int8_t setPruEncoderPos(int32_t );
  int8_t setPwmMicroSeconds(uint8_t const, uint32_t const );
  int8_t setPwmMicroSecondsAll(uint32_t const );
  int8_t setServoNormalized(uint8_t const, float const );
  int8_t setServoNormalizedAll(float const );
  int8_t setEscNormalized(uint8_t const, float const );
  int8_t setEscNormalizedAll(float const );
  int8_t setEscOneshotNormalized(uint8_t const, float const );
  int8_t setEscOneshotNormalizedAll(float const );


  std::vector<Motor> m_motors;
  std::mutex m_mutex;

  unsigned int *m_prusharedMemInt32_ptr;

  std::string const PRU_UNBIND_PATH = "/sys/bus/platform/drivers/pru-rproc/unbind";
  std::string const PRU_BIND_PATH = "/sys/bus/platform/drivers/pru-rproc/bind";
  std::string const PRU0_NAME = "4a334000.pru0";
  std::string const PRU1_NAME = "4a338000.pru1";
  int32_t const PRU_NAME_LEN = 13;
  std::string const PRU0_UEVENT = "/sys/bus/platform/drivers/pru-rproc/4a334000.pru0/uevent";
  std::string const PRU1_UEVENT = "/sys/bus/platform/drivers/pru-rproc/4a338000.pru1/uevent";

  int32_t const PRU_ADDR = 0x4A300000;    // Start of PRU memory Page 184 am335x TRM
  int32_t const PRU_LEN = 0x80000;     // Length of PRU memory
  int32_t const PRU_SHAREDMEM = 0x10000;     // Offset to shared memory
  int32_t const CNT_OFFSET = 64;

  int32_t const SERVO_CHANNELS = 8;
  // Most servos will keep moving out to 600-2400 
  int32_t const SERVO_EXTENDED_RANGE = 1800;
  // normal range is from 900 to 2100 for 120 degree servos
  int32_t const SERVO_NORMAL_RANGE = 1200;
  // servo center at 1500us
  int32_t const SERVO_MID_US = 1500;
  int32_t const PRU_SERVO_LOOP_INSTRUCTIONS = 48;  // instructions per PRU servo timer loop 
};

// 