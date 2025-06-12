#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "commands.hpp"
#include "config.hpp"
#include "process_control_block.hpp"
namespace osemu {
    class Config;
    class Scheduler;
}
namespace osemu {
    void dispatch(Commands cmd,
              std::vector<std::string>& args,
              Config& cfg,
              Scheduler& scheduler);

}