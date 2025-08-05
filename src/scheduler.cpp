#include "scheduler.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <thread>
#include <cmath>

#include "config.hpp"
#include "cpu_worker.h"
#include "process_control_block.hpp"

namespace osemu {

Scheduler::Scheduler() : running_(false), batch_generating_(false), process_counter_(0) {}

Scheduler::~Scheduler() {
  stop();
}

// In scheduler.cpp
// REPLACE the existing dispatch function with this one.
// In scheduler.cpp

void Scheduler::dispatch() {
  while (running_.load()) {
    std::shared_ptr<PCB> process;
    if (!ready_queue_.wait_and_pop(process)) {
      if (running_.load()) continue;
      else break;
    }

    bool dispatched = false;
    while (!dispatched && running_.load()) {
      CpuWorker* idle_worker = nullptr;
      for (auto& worker : cpu_workers_) {
        if (worker->IsIdle()) {
          idle_worker = worker.get();
          break;
        }
      }

      if (!idle_worker) {
        // All cores are busy. Put the process back and wait.
        ready_queue_.push_front(std::move(process));
        // Wait a bit before retrying to avoid a tight busy-loop
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        break; // Exit inner loop to re-evaluate after the sleep.
      }

      // The 'can_dispatch' logic has been removed. If a worker is idle, we can always dispatch.
      // The scheduling algorithm (FCFS vs RR) is handled by the quantum.
      int quantum = (config_.scheduler == SchedulingAlgorithm::FCFS)
                    ? -1
                    : config_.quantumCycles;

      idle_worker->AssignTask(process, quantum);
      dispatched = true;
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
  if(running_.load()) stop();

  running_ = true;
  config_ = config;
  PCB::next_pid = 1;
  process_counter_ = 0;

  memory_manager_ = std::make_unique<MemoryManager>(config_);

  for (uint32_t i = 0; i < config_.cpuCount; ++i) {
    cpu_workers_.push_back(std::make_unique<CpuWorker>(i, *this));
    cpu_workers_.back()->Start();
  }

  std::cout << "Scheduler started with " << config_.cpuCount << " cores." << std::endl;

  dispatch_thread_ = std::thread(&Scheduler::dispatch, this);
  global_clock_thread_ = std::thread(&Scheduler::global_clock, this);

  // --- START THE NEW PAGER THREAD ---
}

// --- MODIFIED stop() FUNCTION ---
void Scheduler::stop() {
  if (!running_.exchange(false)) return;

  if (is_generating()) {
    stop_batch_generation();
  }

  ready_queue_.shutdown();
  clock_cv_.notify_all();

  for (auto& worker : cpu_workers_) {
    worker->Stop();
  }

  if (dispatch_thread_.joinable()) dispatch_thread_.join();
  if (global_clock_thread_.joinable()) global_clock_thread_.join();

  for (auto& worker : cpu_workers_) {
    worker->Join();
  }
  cpu_workers_.clear();

  // Clear all queues and lists
  ready_queue_.empty();
  {
    std::scoped_lock lock(running_mutex_, finished_mutex_, map_mutex_);
    running_processes_.clear();
    finished_processes_.clear();
    all_processes_map_.clear();
  }

  memory_manager_.reset();
  std::cout << "Scheduler stopped." << std::endl;
}


void Scheduler::submit_process(std::shared_ptr<PCB> pcb) {
  if (!running_.load() || !pcb) return;

  // ===================================================================
  // ===================== ADMISSION CONTROL FIX =======================
  // ===================================================================
  if (pcb->getMemorySize() > config_.max_overall_mem) {
    // The process is requesting more memory than the entire system has.
    // It is impossible for it to run. Reject it immediately.
    pcb->terminate("Memory request exceeds total system memory");

    // // Move it directly to finished without bothering the memory manager or ready queue.
    {
      std::scoped_lock lock(finished_mutex_, map_mutex_);
      finished_processes_.push_back(std::move(pcb));
    }
    // std::cout << "Rejected process " << pcb->processName << ": memory request ("
    //           << pcb->getMemorySize() << "B) is larger than total physical memory ("
    //           << config_.max_overall_mem << "B)." << std::endl;
    return; // Stop here. Do not submit to ready queue.
  } else {

  if (memory_manager_) {
    memory_manager_->register_process(pcb);
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    all_processes_map_[pcb->processName] = pcb;
  }
  ready_queue_.push(std::move(pcb));
  }
  // ===================================================================
  // ======================= END OF FIX ================================
  // ===================================================================
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

  std::cout << "----------------------------------------------------------------\n";
  std::cout << "| PROCESS-SMI V01.00   Driver Version: 01.00   |\n";
  std::cout << std::format("CPU Utilization: {:.2f}%\n", cpu_utilization);

  memory_manager_->generate_process_smi_report(std::cout, all_processes_map_);
  std::cout << "----------------------------------------------------------------\n";

  std::cout << "Process Status Details:\n";
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    for (const auto& pcb : running_processes_) {
      std::cout << pcb->status() << std::endl;
    }
  }
  // {
  //   auto ready_copy = ready_queue_.get_copy();
  //   for(const auto& pcb : ready_copy) {
  //       std::cout << pcb->status() << std::endl;
  //   }
  // }
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
  return (it != all_processes_map_.end()) ? it->second : nullptr;
}


// These three functions are critical for clean state management
void Scheduler::move_to_running(std::shared_ptr<PCB> pcb) {
  std::lock_guard<std::mutex> lock(running_mutex_);
  running_processes_.push_back(std::move(pcb));
}

void Scheduler::move_to_finished(std::shared_ptr<PCB> pcb) {
  uint32_t id = pcb->processID;
  {
    std::scoped_lock lock(running_mutex_, finished_mutex_);
    std::erase_if(running_processes_,
                  [&](const auto& p) { return p.get() == pcb.get(); });
    finished_processes_.push_back(std::move(pcb));
  }
  if (memory_manager_) {
    memory_manager_->cleanup_process(id);
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
// In scheduler.cpp
// REPLACE the existing start_batch_generation with this one.

// In scheduler.cpp

void Scheduler::start_batch_generation() {
  if (batch_generating_.exchange(true)) {
    std::cout << "Batch process generation is already running." << std::endl;
    return;
  }

  batch_generator_thread_ = std::make_unique<std::thread>([this]() {
      std::random_device rd;
      std::mt19937 gen(rd());
      // Ensure min <= max before creating distribution
      int min_power = (config_.min_mem_per_proc > 0) ? static_cast<int>(std::log2(config_.min_mem_per_proc)) : 6;
      int max_power = (config_.max_mem_per_proc > 0) ? static_cast<int>(std::log2(config_.max_mem_per_proc)) : 16;
      if (min_power > max_power) std::swap(min_power, max_power);
      std::uniform_int_distribution<> power_dist(min_power, max_power);

      size_t last_generation_tick = 0;

      while (batch_generating_.load()) {
          size_t current_ticks = get_ticks();


          // Check if enough ticks have passed based on the frequency
          if (current_ticks >= last_generation_tick + config_.processGenFrequency) {
              last_generation_tick = current_ticks; // Update the time of last generation

              std::string process_name = "p" + std::to_string(++process_counter_);

              // Roll for memory size
              size_t memory_size = 1 << power_dist(gen);

              auto instructions = instruction_generator_.generateRandomProgram(
                  config_.minInstructions,
                  config_.maxInstructions,
                  process_name,
                  memory_size,
                  config_.mem_per_frame
              );
              // Only submit if instructions were actually generated
              if (instructions.empty()) {
                  // This can happen if memory config is too small to page
                  continue;
              }
              auto pcb = std::make_shared<PCB>(process_name, instructions, memory_size);
              submit_process(pcb);
              // print the process name and memory size
          }

          // Use a short sleep to prevent this thread from busy-waiting and
          // consuming 100% of a real CPU core. This does not affect simulation timing.
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

void Scheduler::calculate_cpu_utilization(size_t& total_cores,
                                          size_t& cores_used,
                                          double& cpu_utilization) const {
  cpu_utilization =
      total_cores > 0 ? (static_cast<double>(cores_used) / total_cores) * 100.0
                      : 0.0;
}

void Scheduler::generate_full_report(const std::string& filename) const {
  std::ofstream report_file(filename);
  if (!report_file.is_open()) {
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
  report_file << "CPU utilization: " << std::fixed << std::setprecision(2) << cpu_utilization << "%\n";
  report_file << "Cores used: " << cores_used << " / " << total_cores << "\n\n";

  if(memory_manager_) {
    report_file << "--- Memory Manager Status ---\n";
    memory_manager_->generate_vmstat_report(report_file);
    report_file << "\n--- Process Memory Details ---\n";
    memory_manager_->generate_process_smi_report(report_file, all_processes_map_);
    report_file << "\n";
  }

  report_file << "--- Process Status ---\n";
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    for (const auto& pcb : running_processes_) {
      report_file << pcb->status() << "\n";
    }
  }
  {
    auto ready_copy = ready_queue_.get_copy();
    for(const auto& pcb : ready_copy) {
        report_file << pcb->status() << "\n";
    }
  }
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