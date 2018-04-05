/*
 * Copyright (C) 2018 Björnborg Ngúyen
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
      0 == commandlineArguments.count("maxvals")){
    std::cerr << argv[0] << " interfaces to the motors of the openKorp drone platform." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --names=<Strings> --types=<esc or servo> --channels=<1...8>  --offsets<-1...1> --maxvals=<floats> [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --names=front-left,front-right,back-right,back-left --types=esc,esc,esc,esc --channels=1,2,3,4 --offsets=0,0,0,0 --maxvals=0,0,0,0 --verbose=2"  << std::endl;
    retCode = 1;
  } else {

    // Setup
    int32_t VERBOSE{commandlineArguments.count("verbose") != 0};
    if (VERBOSE) {
      VERBOSE = std::stoi(commandlineArguments["verbose"]);
    }
    float const FREQ = 50;

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
        (void) envelope;
        // if (envelope.dataType() == opendlv::proxy::GroundSteeringRequest::ID()) {
        //   opendlv::proxy::GroundSteeringRequest gst = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(envelope));
        //   float groundSteering = gst.groundSteering() / angleConversion;
        //   pwmMotors.setMotorPower(1, groundSteering);
        // } else if (envelope.dataType() == opendlv::proxy::PedalPositionRequest::ID()) {
        //   opendlv::proxy::PedalPositionRequest ppr = cluon::extractMessage<opendlv::proxy::PedalPositionRequest>(std::move(envelope));
        //   float val = (ppr.position()+1)/2.0f;
        //   if (val > 1.0f) {
        //     val = 1.0f;
        //   } else if (val < 0.1f){
        //     val = 0.1f;
        //   }
        //   pwmMotors.setMotorPower(2, val);
        // }
      }
    };

    if (VERBOSE == 2) {
      initscr();
    }
    auto atFrequency{[&pwmMotors, &VERBOSE]() -> bool
    {
      // This must be called regularly (>40hz) to keep servos or ESCs awake.
      // pwmMotors.actuate();
      if (VERBOSE == 1) {
        std::cout << pwmMotors.toString() << std::endl;
      }
      if (VERBOSE == 2) {
        mvprintw(1,1,(pwmMotors.toString()).c_str()); 
        refresh();      /* Print it on to the real screen */
      }
      return true;
    }};
    od4.timeTrigger(FREQ, atFrequency);
    if (VERBOSE == 2) {
      endwin();     /* End curses mode      */
    }
  }
  return retCode;
}
