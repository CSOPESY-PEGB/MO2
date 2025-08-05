#include "scheduler.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <thread>

#include "config.hpp"
#include "cpu_worker.h"
#include "process_control_block.hpp"

namespace osemu {


Scheduler::Scheduler() : running_(false), batch_generating_(false), process_counter_(0) {}

Scheduler::~Scheduler() {
  if (batch_generating_.load()) {
    stop_batch_generation();
  }
  if (running_.load()) {
    stop();
  }
}

void Scheduler::dispatch() {
  while (running_.load()) {
    std::shared_ptr<PCB> process;
    if (!ready_queue_.wait_and_pop(process)) { /* ... */ }
    if (!process) continue;

    bool dispatched = false;
    while (!dispatched && running_.load()) {
      CpuWorker* idle_worker = nullptr;
      for (auto& worker : cpu_workers_) {
        if (worker->IsIdle()) { idle_worker = worker.get(); break; }
      }

      if (!idle_worker) {
        ready_queue_.push_front(std::move(process));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        break;
      }

      bool can_dispatch = true;
      if (config_.scheduler == SchedulingAlgorithm::FCFS) {
        // The spec implies FCFS is serialized. The only way to guarantee this
        // behavior without race conditions is to check the running list.
        std::lock_guard<std::mutex> lock(running_mutex_);
        if (!running_processes_.empty()) {
          can_dispatch = false;
        }
      }

      if (can_dispatch) {
        int quantum = (config_.scheduler == SchedulingAlgorithm::FCFS) ? -1 : config_.quantumCycles;
        idle_worker->AssignTask(process, quantum);
        dispatched = true;
      } else {
        // FCFS and another process is already running. Wait.
        ready_queue_.push_front(std::move(process));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
      }
    }
  }
}
void Scheduler::global_clock() {
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (!running_.load()) break;
        {
            std::lock_guard<std::mutex> lock(clock_mutex_);
            ticks_++;
        }
        clock_cv_.notify_all();
    }
}

void Scheduler::start(const Config& config) {
    running_ = true;
    config_ = config;

    memory_manager_ = std::make_unique<MemoryManager>(config_);

    for (uint32_t i = 0; i < config_.cpuCount; ++i) {
        cpu_workers_.push_back(std::make_unique<CpuWorker>(i, *this));
        cpu_workers_.back()->Start();
    }

    std::cout << "Scheduler started with " << config_.cpuCount << " cores." << std::endl;

    dispatch_thread_ = std::thread(&Scheduler::dispatch, this);
    global_clock_thread_ = std::thread(&Scheduler::global_clock, this);
}

void Scheduler::stop() {
    running_ = false;
    ready_queue_.shutdown();
    clock_cv_.notify_all();

    if (dispatch_thread_.joinable()) {
        dispatch_thread_.join();
    }
    if (global_clock_thread_.joinable()) {
        global_clock_thread_.join();
    }
    for (auto& worker : cpu_workers_) {
        worker->Stop();
        worker->Join();
    }
    cpu_workers_.clear();

    memory_manager_.reset();
    std::cout << "Scheduler stopped." << std::endl;
}

void Scheduler::submit_process(std::shared_ptr<PCB> pcb) {
  // Initialize page table for this process if memory manager is available
  if (memory_manager_) {
    // --- NEW LOGIC ---
    // Register the process with the manager so it can have a page table.
    memory_manager_->register_process(pcb);
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    all_processes_map_[pcb->processName] = pcb;
  }
  ready_queue_.push(std::move(pcb));
}

void Scheduler::print_status() const {
  if (!memory_manager_) {
    std::cout << "System not initialized. Cannot print status." << std::endl;
    return;
  }
  double cpu_utilization;
  size_t total_cores = config_.cpuCount;
  size_t cores_used = 0;
  for (const auto& worker : cpu_workers_) {
    if (!worker->IsIdle()) {
      ++cores_used;
    }
  }
  calculate_cpu_utilization(total_cores, cores_used, cpu_utilization);
  std::cout << std::format("CPU Utilization: {:.2f}%\n", cpu_utilization);
  // The memory manager now handles the detailed "process-smi" style report.
  memory_manager_->generate_process_smi_report(std::cout, all_processes_map_);

  std::cout << "----------------------------------------------------------------\n";
  std::cout << "Running processes:\n";
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    for (const auto& pcb : running_processes_) {
      std::cout << pcb->status() << std::endl;
    }
  }
  std::cout << "\nFinished/Terminated processes:\n";
  {
    std::lock_guard<std::mutex> lock(finished_mutex_);
    for (const auto& pcb : finished_processes_) {
      std::cout << pcb->status() << std::endl;
    }
  }
  std::cout << "----------------------------------------------------------------\n";
}

std::shared_ptr<PCB> Scheduler::find_process_by_name(const std::string& processName) const{
  std::lock_guard<std::mutex> lock(map_mutex_);
  
  auto it = all_processes_map_.find(processName);
  
  if (it != all_processes_map_.end()) {
    return it->second;
  }

  return nullptr;
}

void Scheduler::move_to_running(std::shared_ptr<PCB> pcb) {
  std::lock_guard<std::mutex> lock(running_mutex_);
  running_processes_.push_back(pcb);
}

// In scheduler.cpp
void Scheduler::move_to_finished(std::shared_ptr<PCB> pcb) {

  // Create a copy of the ID and Name BEFORE you move the pcb
  // This is a safe way to do it if you must use std::move
  uint32_t id = pcb->processID;
  std::string name = pcb->processName;

  {
    std::scoped_lock lock(running_mutex_, finished_mutex_);

    std::erase_if(running_processes_,
                  [&](const auto& p) { return p.get() == pcb.get(); });

    // The simplest, safest fix is to just copy the shared_ptr.
    // This increments the reference count, which is what shared_ptr is for.
    finished_processes_.push_back(pcb); // REMOVED std::move()
  }

  if (memory_manager_) {
    // Now it is safe to use the pcb variable again, because it was copied, not moved.
    memory_manager_->cleanup_process(pcb->processID);
    std::cout << "Cleaned up memory for process " << pcb->processID << " (" << pcb->processName << ")." << std::endl;
  }
}

void Scheduler::move_to_ready(std::shared_ptr<PCB> pcb) {
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    std::erase_if(running_processes_,
                  [&](const auto& p) { return p.get() == pcb.get(); });
  }

  ready_queue_.push(std::move(pcb));
}

void Scheduler::start_batch_generation() {
  if (batch_generating_.load()) {
    std::cout << "Batch process generation is already running." << std::endl;
    return;
  }

  batch_generating_ = true;
  batch_generator_thread_ = std::make_unique<std::thread>([this]() {
      std::random_device rd;
      std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(config_.min_mem_per_proc, config_.max_mem_per_proc);

      while (batch_generating_.load()) {
          // Logic to generate one process per configured frequency
          std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Simple delay

          std::string process_name;
          {
              std::lock_guard<std::mutex> lock(process_counter_mutex_);
              do {
                  ++process_counter_;
                  process_name = "p" + std::to_string(process_counter_);
              } while (find_process_by_name(process_name) != nullptr);
          }

          size_t memory_size = dist(gen);
          // Ensure it's a power of 2
          memory_size = 1 << static_cast<int>(std::log2(memory_size));

        auto instructions = instruction_generator_.generateRandomProgram(
            config_.minInstructions,
            config_.maxInstructions,
            process_name,
            memory_size,
            config_.mem_per_frame
        );
          // Use the correct 3-argument PCB constructor
          auto pcb = std::make_shared<PCB>(process_name, instructions, memory_size);
          submit_process(pcb);
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

 uint32_t count = 0;
  {
    std::lock_guard<std::mutex> lock(process_counter_mutex_);
    count = process_counter_;
  }
  std::cout << count << " processes generated" << std::endl;
}

void Scheduler::calculate_cpu_utilization(size_t& total_cores,
                                          size_t& cores_used,
                                          double& cpu_utilization) const {

  cpu_utilization =
      total_cores > 0 ? (static_cast<double>(cores_used) / total_cores) * 100.0
                      : 0.0;
}

void Scheduler::generate_full_report(const std::string& filename) const {
  std::ofstream report_file(filename);
  
  if (!report_file) { // Use !is_open() for fstream
    std::cerr << "Error: Could not open report file " << filename << std::endl;
    return;
  }

  double cpu_utilization;
  size_t total_cores = config_.cpuCount;
  size_t cores_used = 0;
  for (const auto& worker : cpu_workers_) {
    if (!worker->IsIdle()) {
      ++cores_used;
    }
  }

  calculate_cpu_utilization(total_cores, cores_used, cpu_utilization);

  report_file << "--- System Utilization Report ---\n";
  report_file << "CPU utilization: " << static_cast<int>(cpu_utilization) << "%\n";
  report_file << "Cores used: " << cores_used << "\n";
  report_file << "Cores available: " << (total_cores - cores_used) << "\n\n";

  // Also include a memory report in the file
  if(memory_manager_) {
    report_file << "--- Memory Manager Status ---\n";
    memory_manager_->generate_process_smi_report(report_file, all_processes_map_);
    report_file << "\n";
  }

  report_file << "--- Process Status ---\n";
  report_file << "Running processes:\n";
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    for (const auto& pcb : running_processes_) {
      report_file << pcb->status() << "\n";
    }
  }

  report_file << "\nFinished/Terminated processes:\n";
  {
    std::lock_guard<std::mutex> lock(finished_mutex_);
    for (const auto& pcb : finished_processes_) {
      report_file << pcb->status() << "\n";
    }
  }

  report_file.close();
  std::cout << "Full system report generated at " << filename << std::endl;
}
}