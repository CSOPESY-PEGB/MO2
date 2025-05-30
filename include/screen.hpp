#pragma once

#include <unordered_set>
#include <vector>
#include <string>

#include "process_control_block.hpp"

namespace osemu {

void screen(std::vector<std::string>& args, 
           std::unordered_set<PCB>& procs, 
           bool& screenSession);

}