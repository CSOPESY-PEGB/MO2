#include "dispatcher.hpp"

#include <iostream>

#include "config.hpp"
#include "console.hpp"
#include "scheduler.hpp"
#include "screen.hpp"

namespace osemu {

void dispatch(Commands cmd, std::vector<std::string>& args, Config& cfg,
              Scheduler& scheduler) {

  static bool initialized = false;
  if(!initialized && cmd != Commands::Initialize){
    std::cout << "Error: System not initialized. Please run `initialize <config_file>` first." << std::endl;
    return;
  }
  switch (cmd) {
    case Commands::Initialize:
      try {
        if(initialized) {
            scheduler.stop();
        }
        cfg = Config::fromFile(args.empty() ? "config.txt" : args[0]);
        scheduler.start(cfg);
        initialized = true;
        std::cout << "System initialized from '" << (args.empty() ? "config.txt" : args[0]) << "'.\n";
      } catch (const std::exception& e) {
        std::cerr << "Error initializing config: " << e.what() << '\n';
        initialized = false;
      }
      break;

    case Commands::Screen:
      screen(args, scheduler, cfg);
      break;

    case Commands::SchedulerStart:
      if (scheduler.is_generating()) {
        std::cout << "Scheduler is already generating processes.\n";
      } else {
        scheduler.start_batch_generation();
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
      scheduler.generate_full_report();
      break;

    case Commands::Clear:
      std::cout << "\x1b[2J\x1b[H";
      console_prompt();
      break;

    case Commands::Exit:
      scheduler.stop();
      break;

    case Commands::ProcessSmi:
      scheduler.print_status();
      break;

    case Commands::Vmstat:
      scheduler.generate_vmstat_report(std::cout);
      break;
  }
}

}