/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ncurses.h>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "PwmMotors.h"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") ||
      0 == commandlineArguments.count("names") ||
      0 == commandlineArguments.count("types") ||
      0 == commandlineArguments.count("channels") ||
      0 == commandlineArguments.count("offsets") ||
      0 == commandlineArguments.count("maxvals")) {
    std::cerr << argv[0] << " interfaces to the motors of the Kiwi platform." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --names=<Strings> --types=<esc or servo> --channels=<1...8>  --offsets<-1...1> --maxvals=<floats> [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --names=steering,propulsion --types=servo,esc --channels=1,2 --offsets=0,0 --maxvals=0.5,0" << std::endl;
    retCode = 1;
  } else {

    // Setup
    std::vector<std::string> names = stringtoolbox::split(commandlineArguments["names"],',');
    std::vector<std::string> types = stringtoolbox::split(commandlineArguments["types"],',');
    std::vector<std::string> channels = stringtoolbox::split(commandlineArguments["channels"],',');
    std::vector<std::string> offets = stringtoolbox::split(commandlineArguments["offsets"],',');
    std::vector<std::string> maxvals = stringtoolbox::split(commandlineArguments["maxvals"],',');

    if (names.size() != types.size() ||
        names.size() != types.size() ||
        names.size() != channels.size() ||
        names.size() != offets.size() ||
        names.size() != maxvals.size()) {
      std::cerr << "Number of arguments do not match, use ',' as delimiter." << std::endl;
      retCode = 1;
    }

    PwmMotors pwmMotors(names, types, channels, offets, maxvals);
    

    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])),
      [&pwmMotors](cluon::data::Envelope &&envelope){
        if (envelope.dataType() == opendlv::proxy::GroundSteeringRequest::ID()) {
          opendlv::proxy::GroundSteeringRequest gst = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(envelope));
          pwmMotors.setMotorPower(1, gst.groundSteering());
        } else if (envelope.dataType() == opendlv::proxy::PedalPositionRequest::ID()) {
          opendlv::proxy::PedalPositionRequest ppr = cluon::extractMessage<opendlv::proxy::PedalPositionRequest>(std::move(envelope));
          float val = (ppr.position()+1)/2.0f;
          if (val > 1.0f) {
            val = 1.0f;
          } else if (val < 0.1f){
            val = 0.1f;
          }
          pwmMotors.setMotorPower(2, val);
        }
      }
    };

    initscr();
    // getch();       /Wait for user input 

    while (od4.isRunning()) {
      // This must be called regularly (>40hz) to keep servos or ESCs awake.
      std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / 50.0f));
      // pwmMotors.actuate();
      mvprintw(1,1,(pwmMotors.toString()).c_str());  /* Print Hello World      */
      // addstr(pwmMotors.toString().c_str());
      refresh();      /* Print it on to the real screen */

    }
    endwin();     /* End curses mode      */
  }
  return retCode;
}
