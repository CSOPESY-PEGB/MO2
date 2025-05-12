#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <stdexcept>

enum class Commands {
    Initialize,
    Screen,
    SchedulerTest,
    SchedulerStop,
    ReportUtil,
    Clear,
    Exit
};

enum class Scheduler {
    FCFS,
    RoundRobin,
};

struct Config {
    uint32_t cpuCount{4};
    Scheduler scheduler{Scheduler::RoundRobin};
    uint32_t quantumCycles{5};
    uint32_t processGenFrequency{1};
    uint32_t minInstructions{1000};
    uint32_t maxInstructions{2000};
    uint32_t delayCyclesPerInstruction{0};

    explicit Config(uint32_t cpu      = 4,
                    Scheduler sched   = Scheduler::RoundRobin,
                    uint32_t quantum  = 5,
                    uint32_t freq     = 1,
                    uint32_t minIns   = 1000,
                    uint32_t maxIns   = 2000,
                    uint32_t delay    = 0)
        : cpuCount            {std::clamp(cpu, 1u, 128u)}
        , scheduler           {sched}
        , quantumCycles       {std::clamp(quantum, 1u, std::numeric_limits<uint32_t>::max())}
        , processGenFrequency {std::clamp(freq, 1u, std::numeric_limits<uint32_t>::max())}
        , minInstructions     {std::clamp(minIns, 1u, std::numeric_limits<uint32_t>::max())}
        , maxInstructions     {std::clamp(maxIns, minInstructions,
                                          std::numeric_limits<uint32_t>::max())}
        , delayCyclesPerInstruction {delay}
    {
        if (scheduler != Scheduler::RoundRobin) { quantumCycles = 1; }
    }

    static Config fromFile(const std::filesystem::path &file) {
        std::ifstream in(file);
        if (!in) throw std::runtime_error("cannot open " + file.string());

        Config cfg;
        std::string key, value;
        while (in >> key >> value) {
            if      (key == "num-cpu")              cfg.cpuCount = std::stoul(value);
            else if (key == "scheduler")            cfg.scheduler =
                       (value == "fcfs") ? Scheduler::FCFS : Scheduler::RoundRobin;
            else if (key == "quantum-cycles")       cfg.quantumCycles = std::stoul(value);
            else if (key == "batch-process-freq")   cfg.processGenFrequency = std::stoul(value);
            else if (key == "min-ins")              cfg.minInstructions = std::stoul(value);
            else if (key == "max-ins")              cfg.maxInstructions = std::stoul(value);
            else if (key == "delay-ins")            cfg.delayCyclesPerInstruction = std::stoul(value);
        }
        return cfg;
    }
};


// TODO: Implement using magic_enum in actual MP. ts pmo frfr mb gng🥀🥀
using CommandMap = std::unordered_map<std::string_view, Commands>;
static const CommandMap cmd_map = {
    {"initialize",    Commands::Initialize},
    {"screen",        Commands::Screen},
    {"scheduler-test",Commands::SchedulerTest},
    {"scheduler-stop",Commands::SchedulerStop},
    {"report-util",   Commands::ReportUtil},
    {"clear",         Commands::Clear},
    {"exit",          Commands::Exit},
};


Commands from_str(const std::string_view cmd) {
    const auto it = cmd_map.find(cmd);
    if (it == cmd_map.end()) {
        throw std::invalid_argument("Unknown command: " + std::string(cmd));
    }
    return it->second;
}

// TODO: Replace with calls to initscr if using ncurses.
void console_prompt() {
    std::cout << R"(
░▒▓███████▓▒░░▒▓████████▓▒░▒▓██████▓▒░░▒▓███████▓▒░
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░     ░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░░▒▓█▓▒░
░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░     ░▒▓█▓▒░      ░▒▓█▓▒░░▒▓█▓▒░
░▒▓███████▓▒░░▒▓██████▓▒░░▒▓█▓▒▒▓███▓▒░▒▓███████▓▒░
░▒▓█▓▒░      ░▒▓█▓▒░     ░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░
░▒▓█▓▒░      ░▒▓█▓▒░     ░▒▓█▓▒░░▒▓█▓▒░▒▓█▓▒░
░▒▓█▓▒░      ░▒▓████████▓▒░▒▓██████▓▒░░▒▓█▓▒░

)";
}

int main() {
    std::string input;
    console_prompt();
    Config cfg;
    while (std::cout << "~ ", std::cin >> input) {
        // Initialize | SchedulerTest | Screen | SchedulerStop | ReportUtil.
        // u guys could refactor this sanay lang talaga ako sa matching pattern
        // sa rust hehe
        try {
            Commands cmd = from_str(input);

            switch (cmd) {
                case Commands::Initialize:
                    cfg = Config::fromFile("config.txt");
                    break;
                case Commands::SchedulerTest:
                    std::cout << input << " command recognized. Doing something." << std::endl;
                    break;
                case Commands::Screen:
                    std::cout << input << " command recognized. Doing something." << std::endl;
                    break;
                case Commands::SchedulerStop:
                    std::cout << input << " command recognized. Doing something." << std::endl;
                    break;
                case Commands::ReportUtil:
                    std::cout << input << " command recognized. Doing something." << std::endl;
                    break;
                case Commands::Clear:
                    std::cout << "\x1b[2J\x1b[H"; // TODO: Implement cls/clear
                    console_prompt();
                    break;
                case Commands::Exit:
                    return 0;
            }
        } catch (const std::invalid_argument& e) {
            std::cerr << e.what() << std::endl;
        }
    }

    return 0;
}