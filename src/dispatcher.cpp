#include "dispatcher.hpp"

#include <iostream>
#include <random>

#include "config.hpp"
#include "console.hpp"
#include "scheduler.hpp"
#include "screen.hpp"

namespace osemu {

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
      if (scheduler.is_generating()) {
        std::cout << "Scheduler is already generating processes.\n";
      } else {
        scheduler.start_batch_generation(cfg);
      }
      break;
      
    case Commands::SchedulerStop:
      if (!scheduler.is_generating()) {
        std::cout << "Scheduler is not currently generating processes.\n";
      } else {
        scheduler.stop_batch_generation();
      }
      break;
    case Commands::ReportUtil:
      scheduler.generate_report();
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




}  // namespace osemu
