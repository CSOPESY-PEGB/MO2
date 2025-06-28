#ifndef OSEMU_DISPATCHER_H_
#define OSEMU_DISPATCHER_H_

#include <string>
#include <vector>

#include "commands.hpp"

namespace osemu {

class Config;
class Scheduler;

void dispatch(Commands cmd, std::vector<std::string>& args, Config& cfg,
              Scheduler& scheduler);

}  

#endif  
