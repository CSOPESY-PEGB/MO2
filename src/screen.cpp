// screen.cpp
#include "screen.hpp"

#include <atomic>  // For std::atomic_bool
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>  // For std::thread and sleep_for
#include <vector>

#include "console.hpp"  // For the prompt after exiting
#include "process_control_block.hpp"
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

// A helper function to handle the 'screen -r' logic
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
  // For the assignment, it's 100 instructions
  auto pcb = std::make_shared<PCB>(process_name, 100);
  scheduler.submit_process(pcb);
}

enum class ScreenCommand { Start, Resume, List, Unknown };

void display_usage() {
  std::cout
      << "Usage:\n"
      << "  screen -s <name>     Start a new process with the given name.\n"
      << "  screen -r <name>     View the real-time log of a running process.\n"
      << "  screen -ls           List all active processes.\n";
}

ScreenCommand parse_command(const std::string& cmd) {
  if (cmd == "-s") return ScreenCommand::Start;
  if (cmd == "-r") return ScreenCommand::Resume;
  if (cmd == "-ls") return ScreenCommand::List;
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
      view_process_log(args[1]);
      break;

    case ScreenCommand::List:
      scheduler.print_status();
      break;

    case ScreenCommand::Unknown:
    default:
      std::cout << "Unknown screen command: " << args[0] << "\n";
      display_usage();
      break;
  }
}

}  // namespace osemu