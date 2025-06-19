// screen.cpp
#include "screen.hpp"

#include <atomic>  // For std::atomic_bool
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>
#include <thread>  // For std::thread and sleep_for
#include <vector>

#include "console.hpp"  // For the prompt after exiting
#include "instruction_generator.hpp"
#include "process_control_block.hpp"
#include "instruction_parser.hpp"
#include "scheduler.hpp"

namespace osemu {
namespace {

// This function will be run in a separate thread to watch the log file
void tail_log_file(const std::string& filename, std::atomic<bool>& should_run) {
  std::ifstream log_file(filename);
  if (!log_file.is_open()) {
    return;  // File might not exist yet, thread will just exit.
  }

  // Move to the end of the file initially, but we will print everything first.
  // So we'll read from the start, then seek to the end.

  std::string line;
  // Go to the beginning to read existing content
  log_file.seekg(0, std::ios::beg);
  while (std::getline(log_file, line)) {
    std::cout << line << std::endl;
  }

  // Loop as long as the main thread says we should run
  while (should_run.load()) {
    // Read any new lines
    while (std::getline(log_file, line)) {
      std::cout << line << std::endl;
    }

    // If we reached the end of the file, we need to clear the EOF flag
    // so we can detect future writes.
    if (log_file.eof()) {
      log_file.clear();
    }

    // Wait a bit before checking the file again to avoid burning CPU
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// Helper function to find a process by name
std::shared_ptr<PCB> find_process(const std::string& process_name, Scheduler& scheduler) {
  // This is a simplified implementation - in a real system we'd need
  // a better way to access processes from the scheduler
  // For now, return nullptr to indicate not implemented
  return nullptr;
}

// A helper function to handle the 'screen -r' logic
void view_process_screen(const std::string& process_name, Scheduler& scheduler) {
  // Clear the screen and enter process view mode
  std::cout << "\x1b[2J\x1b[H";
  
  std::string input_line;
  while (true) {
    std::cout << "Process name: " << process_name << std::endl;
    std::cout << "ID: 1" << std::endl;  // Simplified for now
    std::cout << "Logs:" << std::endl;
    
    // Try to read from log file if it exists
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
    std::cout << "Current instruction line: N/A" << std::endl;  // Would need PCB access
    std::cout << "Lines of code: N/A" << std::endl;
    std::cout << std::endl;
    
    std::cout << "root:\\> ";
    if (!std::getline(std::cin, input_line)) {
      break;
    }
    
    if (input_line == "exit") {
      break;
    } else if (input_line == "process-smi") {
      // Refresh the display - continue the loop
      std::cout << "\x1b[2J\x1b[H";
      continue;
    } else {
      std::cout << "Unknown command: " << input_line << std::endl;
      std::cout << "Available commands: process-smi, exit" << std::endl;
    }
  }
  
  // Restore the main console
  std::cout << "\x1b[2J\x1b[H";
  console_prompt();
}

// A helper function to handle the 'screen -r' logic (legacy file viewing)
void view_process_log(const std::string& process_name) {
  std::string filename = process_name + ".txt";

  // Check if the log file exists. It might not if the process hasn't run yet.
  if (!std::filesystem::exists(filename)) {
    std::cout << "Log file for process '" << process_name
              << "' not found. The process may not have started writing yet."
              << std::endl;
    return;
  }

  // Clear the main screen
  std::cout << "\x1b[2J\x1b[H";
  std::cout << "--- Viewing log for '" << process_name
            << "'. Type 'exit' to return. ---" << std::endl;

  // Create a flag to communicate with the tailer thread
  std::atomic<bool> tailer_running = true;

  // Start the thread that will watch and print the log file
  std::thread tailer_thread(tail_log_file, filename, std::ref(tailer_running));

  // Meanwhile, the main thread will block here, waiting for user input
  std::string input_line;
  while (std::cout << process_name << "> " << std::flush &&
         std::getline(std::cin, input_line)) {
    if (input_line == "exit") {
      break;
    }
  }

  // Signal the tailer thread that it's time to stop
  tailer_running = false;

  // Wait for the tailer thread to finish its last loop and exit cleanly
  if (tailer_thread.joinable()) {
    tailer_thread.join();
  }

  // Restore the main console
  std::cout << "\x1b[2J\x1b[H";
  console_prompt();
}

// Helper to create a process
void create_process(const std::string& process_name, Scheduler& scheduler) {
  InstructionGenerator generator;
  // Generate a reasonable number of instructions for manual testing (20-50)
  auto instructions = generator.generateRandomProgram(20, 50, process_name);
  auto pcb = std::make_shared<PCB>(process_name, instructions);
  
  std::cout << "Created process '" << process_name << "' with " 
            << instructions.size() << " instructions." << std::endl;
  
  scheduler.submit_process(pcb);
}

// Helper to create a process from .opesy file
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

}  // end anonymous namespace

// This is the main function called by your dispatcher
void screen(std::vector<std::string>& args, Scheduler& scheduler) {
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
      create_process(args[1], scheduler);
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

}  // namespace osemu