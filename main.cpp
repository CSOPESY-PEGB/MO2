#include <iostream>
#include <string>

enum class Commands {
  Initialize,
  Screen,
  SchedulerTest,
  SchedulerStop,
  ReportUtil,
  Clear,
  Exit
};

// TODO: Implement using magic_enum in actual MP. ts pmo frfr mb gng🥀🥀
static const std::unordered_map<std::string, Commands> cmd_map = {
    {"initialize",       Commands::Initialize},
    {"screen",           Commands::Screen},
    {"scheduler-test",   Commands::SchedulerTest},
    {"scheduler-stop",   Commands::SchedulerStop},
    {"report-util",      Commands::ReportUtil},
    {"clear",            Commands::Clear},
    {"exit",             Commands::Exit},
};

Commands from_str(const std::string& s) {
    auto it = cmd_map.find(s);
    if (it != cmd_map.end()) return it->second;
    throw std::invalid_argument("Unknown command");
}

int main(int argc, char** argv) {
  while (1) {
    std::string input;
    std::cout << "~ ", std::cin >> input; // dumb ahh term prompt
    return 0;   
  }
}

