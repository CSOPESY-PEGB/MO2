#include "scheduler.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include "config.hpp"
#include "process_control_block.hpp"

namespace osemu {

class Scheduler::CPUWorker {
 public:
  CPUWorker(int core_id, Scheduler& scheduler, std::atomic<bool>& running)
      : core_id_(core_id), scheduler_(scheduler), system_running_(running) {}

  void start() { thread_ = std::thread(&CPUWorker::run, this); }

  void join() {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

 private:
  void run() {
    while (true) {
      std::shared_ptr<PCB> pcb;

      scheduler_.ready_queue_.wait_and_pop(pcb);

      if (!system_running_ || !pcb) {
        break;
      }

      execute_process(pcb);
    }
  }

  void execute_process(std::shared_ptr<PCB> pcb) {
    pcb->assignedCore = core_id_;
    scheduler_.move_to_running(pcb);
    std::ofstream log_file(pcb->processName + ".txt");

    while (!pcb->isComplete()) {
      // Clear previous logs to get only new output from this step
      const auto& logs_before = pcb->getExecutionLogs();
      size_t logs_count_before = logs_before.size();
      
      pcb->step();
      
      // Get new logs produced by this step
      const auto& logs_after = pcb->getExecutionLogs();
      for (size_t i = logs_count_before; i < logs_after.size(); ++i) {
        log_file << logs_after[i] << " Core:" << core_id_ << std::endl;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    pcb->finishTime = std::chrono::system_clock::now();
    scheduler_.move_to_finished(pcb);
  }
  int core_id_;
  std::thread thread_;
  Scheduler& scheduler_;
  std::atomic<bool>& system_running_;
};

Scheduler::Scheduler() : running_(false), batch_generating_(false), process_counter_(0) {}

Scheduler::~Scheduler() {
  if (batch_generating_.load()) {
    stop_batch_generation();
  }
  if (running_.load()) {
    stop();
  }
}

void Scheduler::start(const Config& config) {
  running_ = true;
  for (uint32_t i = 0; i < config.cpuCount; ++i) {
    cpu_workers_.push_back(std::make_unique<CPUWorker>(i, *this, running_));
    cpu_workers_.back()->start();
  }
  std::cout << "Scheduler started with " << config.cpuCount << " cores."
            << std::endl;
}

void Scheduler::stop() {
  if (batch_generating_.load()) {
    stop_batch_generation();
  }

  if (!running_.exchange(false)) {
    return;
  }

  for (size_t i = 0; i < cpu_workers_.size(); ++i) {
    ready_queue_.push(nullptr);
  }

  for (auto& worker : cpu_workers_) {
    worker->join();
  }
  cpu_workers_.clear();
  std::cout << "Scheduler stopped." << std::endl;
}

void Scheduler::submit_process(std::shared_ptr<PCB> pcb) {
  std::cout << "Process " << pcb->processName << " submitted to scheduler."
            << std::endl;
  ready_queue_.push(std::move(pcb));
}

void Scheduler::print_status() const {
  std::cout
      << "----------------------------------------------------------------\n";
  std::cout << "Running processes:\n";
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    for (const auto& pcb : running_processes_) {
      std::cout << pcb->status() << std::endl;
    }
  }

  std::cout << "\nFinished processes:\n";
  {
    std::lock_guard<std::mutex> lock(finished_mutex_);
    for (const auto& pcb : finished_processes_) {
      std::cout << pcb->status() << std::endl;
    }
  }
  std::cout
      << "----------------------------------------------------------------\n";
}

void Scheduler::move_to_running(std::shared_ptr<PCB> pcb) {
  std::lock_guard<std::mutex> lock(running_mutex_);
  running_processes_.push_back(std::move(pcb));
}

void Scheduler::move_to_finished(std::shared_ptr<PCB> pcb) {
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    std::erase_if(running_processes_, [&](const auto& p) {
      return p->processName == pcb->processName;
    });
  }
  {
    std::lock_guard<std::mutex> lock(finished_mutex_);
    finished_processes_.push_back(std::move(pcb));
  }
}

void Scheduler::start_batch_generation(const Config& config) {
  if (batch_generating_.load()) {
    std::cout << "Batch process generation is already running." << std::endl;
    return;
  }
  
  batch_generating_ = true;
  batch_generator_thread_ = std::make_unique<std::thread>([this, config]() {
    uint32_t cpu_cycles = 0;
    
    while (batch_generating_.load()) {
      cpu_cycles++;
      
      // Generate a new process every batch_process_freq cycles
      if (cpu_cycles % config.processGenFrequency == 0) {
        ++process_counter_;
        std::string process_name = "p" + std::string(2 - std::to_string(process_counter_).length(), '0') + std::to_string(process_counter_);
        
        // Generate random instructions based on config
        auto instructions = instruction_generator_.generateRandomProgram(
          config.minInstructions, 
          config.maxInstructions, 
          process_name
        );
        
        auto pcb = std::make_shared<PCB>(process_name, instructions);
        submit_process(pcb);
        
        std::cout << "Generated process: " << process_name << " with " 
                  << instructions.size() << " instructions." << std::endl;
      }
      
      // CPU cycle timing (simplified)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  
  std::cout << "Started batch process generation." << std::endl;
}

void Scheduler::stop_batch_generation() {
  if (!batch_generating_.exchange(false)) {
    return;
  }
  
  if (batch_generator_thread_ && batch_generator_thread_->joinable()) {
    batch_generator_thread_->join();
  }
  batch_generator_thread_.reset();
  
  std::cout << "Stopped batch process generation." << std::endl;
}

void Scheduler::generate_report(const std::string& filename) const {
  std::ofstream report_file(filename);
  
  if (!report_file.is_open()) {
    std::cerr << "Failed to create report file: " << filename << std::endl;
    return;
  }
  
  // Calculate CPU utilization
  size_t total_cores = cpu_workers_.size();
  size_t cores_used = 0;
  
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    cores_used = running_processes_.size();
  }
  
  double cpu_utilization = total_cores > 0 ? (static_cast<double>(cores_used) / total_cores) * 100.0 : 0.0;
  
  report_file << "CPU utilization: " << static_cast<int>(cpu_utilization) << "%\n";
  report_file << "Cores used: " << cores_used << "\n";
  report_file << "Cores available: " << (total_cores - cores_used) << "\n\n";
  
  report_file << "Running processes:\n";
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    for (const auto& pcb : running_processes_) {
      report_file << pcb->status() << "\n";
    }
  }
  
  report_file << "\nFinished processes:\n";
  {
    std::lock_guard<std::mutex> lock(finished_mutex_);
    for (const auto& pcb : finished_processes_) {
      report_file << pcb->status() << "\n";
    }
  }
  
  report_file.close();
  std::cout << "Report generated at " << filename << "!" << std::endl;
}

}  // namespace osemu
