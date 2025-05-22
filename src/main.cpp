#include <iostream>

#include "commands.hpp"
#include "config.hpp"
#include "console.hpp"
#include "dispatcher.hpp"
#include "parser.hpp"
#include "process_control_block.hpp"

int main() {
  Config cfg;
  std::unordered_set<PCB> processes;
  console_prompt();
  bool screenSession = false;
  std::string line;
  while (std::cout << "~ " && std::getline(std::cin, line)) {
    auto tokens = parse_tokens(line);
    if (tokens.empty()) continue;

    try {
      Commands cmd = from_str(tokens.front());
      tokens.erase(tokens.begin());
      dispatch(cmd, tokens, cfg, processes, screenSession);
    } catch (const std::exception& ex) {
      std::cerr << ex.what() << '\n';
    }
  }
  return 0;
}
