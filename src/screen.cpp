
#include "screen.hpp"

#include <atomic>  
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>
#include <thread>  
#include <vector>

#include "console.hpp"  
#include "instruction_generator.hpp"
#include "process_control_block.hpp"
#include "instruction_parser.hpp"
#include "scheduler.hpp"

namespace osemu {
namespace {





std::shared_ptr<PCB> find_process(const std::string& process_name,
                                  Scheduler& scheduler) {
  if (scheduler.find_process_by_name(process_name) != nullptr) {
    return scheduler.find_process_by_name(process_name);
  }

  return nullptr;
}


void view_process_screen(const std::string& process_name, Scheduler& scheduler) {
  // scheduler gets the process
  try {
    std::shared_ptr<PCB> pcb = find_process(process_name, scheduler);

    if (!pcb) {
      throw std::runtime_error("Process " + process_name + " not found.");
    }
    std::cout << "\x1b[2J\x1b[H";


    std::string input_line;
    while (true) {
      std::cout << "Process name: " << process_name << std::endl;
      std::cout << "ID: "  << pcb->processID<< std::endl;
      std::cout << "Logs:" << std::endl;

      const auto& logs = pcb->getExecutionLogs();
      if (logs.empty()) {
        std::cout << "(No logs yet)" << std::endl;
      } else {
        for (const auto& log : logs) {
          std::cout << log << std::endl;
        }
      }

      std::cout << std::endl;
      std::cout << "Current instruction line: "<< pcb->currentInstruction << std::endl;
      std::cout << "Lines of code: " << pcb-> totalInstructions << std::endl;
      std::cout << std::endl;

      std::cout << "root:\\> ";
      if (!std::getline(std::cin, input_line)) {
        break;
      }

      if (input_line == "exit") {
        break;
      } else if (input_line == "process-smi") {

        std::cout << "\x1b[2J\x1b[H";
        continue;
      } else {
        std::cout << "Unknown command: " << input_line << std::endl;
        std::cout << "Available commands: process-smi, exit" << std::endl;
      }
    }


    std::cout << "\x1b[2J\x1b[H";
    console_prompt();
  }
  catch (const std::exception& e) {
    std::cout << e.what() << std::endl;

  }

}





void create_process(const std::string& process_name, Scheduler& scheduler, Config& config) {
  InstructionGenerator generator;

  auto instructions = generator.generateRandomProgram(config.minInstructions, config.maxInstructions, process_name);
  auto pcb = std::make_shared<PCB>(process_name, instructions);
  
  std::cout << "Created process '" << process_name << "' with " 
            << instructions.size() << " instructions." << std::endl;
  
  scheduler.submit_process(pcb);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

}


void create_process_from_file(const std::string& filename, const std::string& process_name, Scheduler& scheduler) {
  std::ifstream file(filename);
  if (!file) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return;
  }
  
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string input = buffer.str();
  
  std::vector<Expr> program;
  ParseResult result = InstructionParser::parse_program(input, program);
  
  if (!result.success) {
    std::cerr << "Parse error: " << result.error_msg << std::endl;
    std::cerr << "Remaining input: " << result.remaining << std::endl;
    return;
  }
  
  auto pcb = std::make_shared<PCB>(process_name, program);
  
  std::cout << "Created process '" << process_name << "' from file '" << filename 
            << "' with " << program.size() << " instructions." << std::endl;
  
  scheduler.submit_process(pcb);
}

enum class ScreenCommand { Start, Resume, List, File, Unknown };

void display_usage() {
  std::cout
      << "Usage:\n"
      << "  screen -s <name>     Start a new process with the given name.\n"
      << "  screen -r <name>     View the real-time log of a running process.\n"
      << "  screen -ls           List all active processes.\n"
      << "  screen -f <file> <name>  Load process from .opesy file.\n";
}

ScreenCommand parse_command(const std::string& cmd) {
  if (cmd == "-s") return ScreenCommand::Start;
  if (cmd == "-r") return ScreenCommand::Resume;
  if (cmd == "-ls") return ScreenCommand::List;
  if (cmd == "-f") return ScreenCommand::File;
  return ScreenCommand::Unknown;
}

}  


void screen(std::vector<std::string>& args, Scheduler& scheduler, Config& config) {
  if (args.empty()) {
    display_usage();
    return;
  }

  const ScreenCommand cmd = parse_command(args[0]);

  switch (cmd) {
    case ScreenCommand::Start:
      if (args.size() != 2) {
        display_usage();
        return;
      }
      create_process(args[1], scheduler, config);
      view_process_screen(args[1],scheduler);
      break;

    case ScreenCommand::Resume:
      if (args.size() != 2) {
        display_usage();
        return;
      }
      view_process_screen(args[1], scheduler);
      break;

    case ScreenCommand::List:
      scheduler.print_status();
      break;

    case ScreenCommand::File:
      if (args.size() != 3) {
        display_usage();
        return;
      }
      create_process_from_file(args[1], args[2], scheduler);
      break;

    case ScreenCommand::Unknown:
    default:
      std::cout << "Unknown screen command: " << args[0] << "\n";
      display_usage();
      break;
  }
}

}  
