#include "dispatcher.hpp"

#include <iostream>
#include <random>

#include "config.hpp"
#include "console.hpp"
#include "scheduler.hpp"
#include "screen.hpp"

namespace osemu {
void generate_test_processes(Scheduler& scheduler, const Config& cfg);

void dispatch(Commands cmd, std::vector<std::string>& args, Config& cfg,
              Scheduler& scheduler) {
  switch (cmd) {
    case Commands::Initialize:
      try {
        scheduler.stop();
        cfg = Config::fromFile(args.empty() ? "../config.txt" : args[0]);
        scheduler.start(cfg);
        std::cout << "System initialized from '" << (args.empty() ? "config.txt" : args[0]) << "'.\n";

      } catch (const std::exception& e) {
        std::cerr << "Error initializing config: " << e.what() << '\n';
      }
      break;

    case Commands::Screen:
      screen(args, scheduler);
      break;

    case Commands::SchedulerStart:
      std::cout << "Loading test workload...\n";
      generate_test_processes(scheduler, cfg);
      break;
    case Commands::SchedulerStop:
    case Commands::ReportUtil:
      std::cout << "TODO: Implement command '" << static_cast<int>(cmd)
                << "'\n";
      break;

    case Commands::Clear:
      std::cout << "\x1b[2J\x1b[H";
      console_prompt();
      break;

    case Commands::Exit:
      scheduler.stop();
      break;
  }
}
void generate_test_processes(osemu::Scheduler& scheduler, const Config& cfg) {
  const int process_count = 10;

  std::cout << "Generating " << process_count << " test processes...\n";

  // Set up random number generation
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distrib(cfg.minInstructions, cfg.maxInstructions);

  for (int i = 1; i <= process_count; ++i) {
    size_t instructions = distrib(gen);
    std::string name = std::format("process{:02}", i);

    auto pcb = std::make_shared<osemu::PCB>(name, instructions);
    std::cout << "  -> Submitting " << name << " with " << instructions
              << " instructions.\n";
    scheduler.submit_process(pcb);
  }
}



}  // namespace osemu
