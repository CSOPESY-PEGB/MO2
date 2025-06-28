#ifndef OSEMU_SCREEN_H_
#define OSEMU_SCREEN_H_

#include <string>
#include <vector>

#include "config.hpp"

namespace osemu {

class Scheduler;

void screen(std::vector<std::string>& args, Scheduler& scheduler, Config& config);

}  

#endif  
