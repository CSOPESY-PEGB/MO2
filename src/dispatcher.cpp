#include "dispatcher.hpp"
#include <iostream>
#include "config.hpp"
#include "console.hpp"
#include "screen.hpp"
#include "scheduler.hpp" 

namespace osemu {
    void dispatch(Commands cmd,
                  std::vector<std::string>& args,
                  Config& cfg,
                  Scheduler& scheduler) {
        switch (cmd) {
            case Commands::Initialize:
                try {
                    cfg = Config::fromFile(args.empty() ? "config.txt" : args[0]);
                } catch (const std::exception& e) {
                    std::cerr << "Error initializing config: " << e.what() << '\n';
                }
                break;

            case Commands::Screen:
                screen(args, scheduler);
                break;

            case Commands::SchedulerTest:
            case Commands::SchedulerStop:
            case Commands::ReportUtil:
                std::cout << "TODO: Implement command '" << static_cast<int>(cmd) << "'\n";
                break;

            case Commands::Clear:
                std::cout << "\x1b[2J\x1b[H";
                console_prompt();
                break;

            case Commands::Exit:
                
                break;
        }
    }

} 
