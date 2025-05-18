#include "dispatcher.hpp"
#include "console.hpp"
#include <cstdlib>
#include <iostream>

void dispatch(Commands cmd,
              const std::vector<std::string>& args,
              Config& cfg)
{
    switch (cmd) {
        case Commands::Initialize:
            cfg = Config::fromFile(args.empty() ? "config.txt" : args[0]);
            break;
        case Commands::SchedulerTest:
        case Commands::Screen:

        case Commands::SchedulerStop:
        case Commands::ReportUtil:
            std::cout << "TODO run " << static_cast<int>(cmd) << '\n';
            break;

        case Commands::Clear:
            std::cout << "\x1b[2J\x1b[H";
            console_prompt();
            break;

        case Commands::Exit:
            std::exit(0);
    }
}
