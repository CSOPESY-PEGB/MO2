
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


void tail_log_file(const std::string& filename, std::atomic<bool>& should_run) {
  std::ifstream log_file(filename);
  if (!log_file.is_open()) {
    return;  
  }

  
  

  std::string line;
  
  log_file.seekg(0, std::ios::beg);
  while (std::getline(log_file, line)) {
    std::cout << line << std::endl;
  }

  
  while (should_run.load()) {
    
    while (std::getline(log_file, line)) {
      std::cout << line << std::endl;
    }

    
    
    if (log_file.eof()) {
      log_file.clear();
    }

    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}


std::shared_ptr<PCB> find_process(const std::string& process_name, Scheduler& scheduler) {

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


      std::string filename = process_name + ".txt";
      std::ifstream log_file(filename);
      if (log_file.is_open()) {
        std::string line;
        while (std::getline(log_file, line)) {
          std::cout << line << std::endl;
        }
        log_file.close();
      } else {
        std::cout << "(No logs yet)" << std::endl;
      }

      std::cout << std::endl;
      std::cout << "Current instruction line: N/A" << std::endl;
      std::cout << "Lines of code: N/A" << std::endl;
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


void view_process_log(const std::string& process_name) {
  std::string filename = process_name + ".txt";

  
  if (!std::filesystem::exists(filename)) {
    std::cout << "Log file for process '" << process_name
              << "' not found. The process may not have started writing yet."
              << std::endl;
    return;
  }

  
  std::cout << "\x1b[2J\x1b[H";
  std::cout << "--- Viewing log for '" << process_name
            << "'. Type 'exit' to return. ---" << std::endl;

  
  std::atomic<bool> tailer_running = true;

  
  std::thread tailer_thread(tail_log_file, filename, std::ref(tailer_running));

  
  std::string input_line;
  while (std::cout << process_name << "> " << std::flush &&
         std::getline(std::cin, input_line)) {
    if (input_line == "exit") {
      break;
    }
  }

  
  tailer_running = false;

  
  if (tailer_thread.joinable()) {
    tailer_thread.join();
  }

  
  std::cout << "\x1b[2J\x1b[H";
  console_prompt();
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
