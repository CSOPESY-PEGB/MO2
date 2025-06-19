#ifndef OSEMU_COMMANDS_H_
#define OSEMU_COMMANDS_H_

#include <string_view>
#include <unordered_map>

namespace osemu {

enum class Commands {
  Initialize,
  Screen,
  SchedulerStart,
  SchedulerStop,
  ReportUtil,
  Clear,
  Exit
};

Commands from_str(std::string_view cmd);

}  // namespace osemu

#endif  // OSEMU_COMMANDS_H_
