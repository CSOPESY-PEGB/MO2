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
      scheduler.start_generator(cfg);
      break;
    case Commands::SchedulerStop:
      scheduler.stop_generator();
      break;

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




}  // namespace osemu
