#include "commands.hpp"

#include <stdexcept>
#include <string>

namespace osemu {

namespace {
using CommandMap = std::unordered_map<std::string_view, Commands>;
const CommandMap cmd_map{
    {"initialize", Commands::Initialize},
    {"screen", Commands::Screen},
    {"scheduler-start", Commands::SchedulerStart},
    {"scheduler-stop", Commands::SchedulerStop},
    {"report-util", Commands::ReportUtil},
    {"clear", Commands::Clear},
    {"exit", Commands::Exit},

};
}

Commands from_str(std::string_view cmd) {
  const auto it = cmd_map.find(cmd);
  if (it == cmd_map.end()) {
    throw std::invalid_argument("Unknown command: " + std::string{cmd});
  }
  return it->second;
}

}
