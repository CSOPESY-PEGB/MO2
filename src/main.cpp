#include <iostream>
#include <memory>
#include <random>
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

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distrib(100, 500);

  Config cfg;
  cfg.cpuCount = 4;
  cfg.scheduler = SchedulingAlgorithm::FCFS;

  Scheduler scheduler;
  console_prompt();

  scheduler.start(cfg);

  for (int i = 1; i <= 10; ++i) {
    std::string name =
        std::string("process") + (i < 10 ? "0" : "") + std::to_string(i);
    size_t instructions = distrib(gen);
    auto pcb = std::make_shared<PCB>(name, instructions);
    std::cout << "  -> Created " << name << " with " << instructions
              << " instructions." << std::endl;
    scheduler.submit_process(pcb);
  }

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
