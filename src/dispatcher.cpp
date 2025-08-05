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

  static bool initialized = false;
  if(!initialized && cmd != Commands::Initialize){
    std::cout << "no config loaded, please call `initialize` on a config file." << std::endl;
    return;
  }
  switch (cmd) {
    case Commands::Initialize:
      try {
        initialized = true;
        scheduler.stop();
        cfg = Config::fromFile(args.empty() ? "config.txt" : args[0]);
        scheduler.start(cfg);

        std::cout << "System initialized from '" << (args.empty() ? "config.txt" : args[0]) << "'.\n";

      } catch (const std::exception& e) {
        std::cerr << "Error initializing config: " << e.what() << '\n';
      }
      break;

    case Commands::Screen:
      screen(args, scheduler, cfg);
      break;

    case Commands::SchedulerStart:
      // --- FIX ---
      // Call the correct public method.
      if (scheduler.is_generating()) {
        std::cout << "Scheduler is already generating processes.\n";
      } else {
        // The function now takes no arguments.
        scheduler.start_batch_generation();
      }
      break;
      
    case Commands::SchedulerStop:
      // --- FIX ---
      // Call the correct public method.
      if (!scheduler.is_generating()) {
        std::cout << "Scheduler is not currently generating processes.\n";
      } else {
        scheduler.stop_batch_generation();
      }
      break;
      
    case Commands::ReportUtil:
      // --- FIX ---
      // This command should now generate a detailed report file.
      // A simple status to the console is handled by 'screen -ls' or 'process-smi'.
      // Let's create a new 'generate_full_report' function for this.
      scheduler.generate_full_report(); // We will create this new function.
      break;


    case Commands::Clear:
      std::cout << "\x1b[2J\x1b[H";
      console_prompt();
      break;

    case Commands::Exit:
      scheduler.stop();
      break;
      
    case Commands::ProcessSmi:
      scheduler.print_status(); // The 'screen -ls' command is now the process-smi
      break;
      
    case Commands::Vmstat:
      if (scheduler.get_memory_manager()) {
        scheduler.get_memory_manager()->generate_vmstat_report(std::cout);
      } else {
        std::cout << "Memory manager not initialized." << std::endl;
      }
      break;
  }
}




}
