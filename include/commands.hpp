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
  ProcessSmi,
  Vmstat,
  Clear,
  Exit
};

Commands from_str(std::string_view cmd);

}

#endif
