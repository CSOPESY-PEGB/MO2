#pragma once

#include <string_view>
#include <unordered_map>

namespace osemu {

enum class Commands {
    Initialize,
    Screen,
    SchedulerTest,
    SchedulerStop,
    ReportUtil,
    Clear,
    Exit
};

Commands from_str(std::string_view cmd);

}