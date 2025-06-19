#include <iostream>
#include <memory>
#include <string>

#include "commands.hpp"
#include "config.hpp"
#include "console.hpp"
#include "dispatcher.hpp"
#include "parser.hpp"
#include "process_control_block.hpp"
#include "scheduler.hpp"


int main() {
  using namespace osemu;


  Config cfg;
  Scheduler scheduler;

  console_prompt();

  std::string line;
  while (std::cout << "~ " << std::flush && std::getline(std::cin, line)) {
    auto tokens = ParseTokens(line);
    if (tokens.empty()) {
      continue;
    }

    try {
      Commands cmd = from_str(tokens.front());

      if (cmd == Commands::Exit) {
        break;
      }

      tokens.erase(tokens.begin());
      dispatch(cmd, tokens, cfg, scheduler);
    } catch (const std::exception& ex) {
      std::cerr << "Error: " << ex.what() << '\n';
    }
  }

  std::cout << "Emulator has shut down cleanly." << std::endl;
  return 0;
}
