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

void Scheduler::dispatch(){
  while(running_.load()){
    std::shared_ptr<PCB> process;
    
    if(!ready_queue_.wait_and_pop(process)){
      if(!running_.load()) break;
      else continue;
    }

    if(!process){
      if(!running_.load()) break;
      else continue;
    }
    
    // Check with the memory manager, the single source of truth.
    bool can_run = false;
    if (memory_manager_) {
        if (memory_manager_->is_allocated(process->processID)) {
            // It's already in memory, so it can run.
            can_run = true;
        } else {
            // It's not in memory, try to allocate it.
            if (memory_manager_->allocate(process->processID, mem_per_proc_, process)) {
                // Allocation succeeded, so it can run.
                can_run = true;
            }
            // If allocation fails, can_run remains false.
        }
    }
    
    if (!can_run) {
        // Defer the process if it couldn't be run.
        ready_queue_.push(std::move(process));
        continue;
    }
    
    // If we reach here, the process has memory and is ready for a CPU.
    bool dispatched = false;
    while (!dispatched && running_.load()){
      for (auto& worker: cpu_workers_){
        if (worker->IsIdle()){
          
          if (algorithm_ == SchedulingAlgorithm::FCFS) {
            int remaining_instructions = -1;
            worker->AssignTask(process, remaining_instructions);
            dispatched = true;
            break;
          } else if (algorithm_ == SchedulingAlgorithm::RoundRobin) {
            int remaining_instructions = process->totalInstructions - process->currentInstruction;
            int steps_to_run = std::min((int)quantum_cycles_, remaining_instructions);
            worker->AssignTask(process, steps_to_run);
            dispatched = true;
            break;
          }
        }
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

    if (running_.load() && memory_manager_ &&
        (ticks_.load() % quantum_cycles_) == 0 && ticks_.load() > 0) {
      quantum_report_counter_++;
    }
  }
}

void Scheduler::check_config_scheduler_for_discrepancies(const Config& config) {
  std::cout << "debug: max_overall_mem " << config.max_overall_mem << std::endl;
  std::cout << "debug: maxOverallMem " << maxOverallMemory << std::endl;

  std::cout << "debug: mem_per_frame (config)" << config.mem_per_frame
            << std::endl;
  std::cout << "debug: memPerFrame (scheduler)" << memPerFrame << std::endl;

  std::cout << "debug: mem_per_proc_ (scheduler) " << mem_per_proc_
            << std::endl;
  std::cout << "debug: mem_per_proc_ (config) " << config.min_mem_per_proc
            << std::endl;

  std::cout << "debug: minMemPerProc - (scheduler) " << minMemPerProc
            << std::endl;
  std::cout << "debug: min_mem_per_proc - (config) " << config.min_mem_per_proc
            << std::endl;

  std::cout << "debug: maxMemPerProc - scheduler " << maxMemPerProc
            << std::endl;
  std::cout << "debug: max_mem_per_proc - config " << config.max_mem_per_proc
            << std::endl;
}
void Scheduler::start(const Config& config) {
  running_ = true;
  delay_per_exec_ = config.delayCyclesPerInstruction;
  quantum_cycles_ = config.quantumCycles;
  algorithm_ = config.scheduler;
  core_count_ = config.cpuCount;
  maxOverallMemory = config.max_overall_mem;
  memPerFrame = config.mem_per_frame;
  minMemPerProc = config.min_mem_per_proc;
  maxMemPerProc = config.max_mem_per_proc;

  memory_manager_ = std::make_unique<MemoryManager>(config.max_overall_mem, config.mem_per_frame);
  mem_per_proc_ = config.min_mem_per_proc;

  for (uint32_t i = 0; i < config.cpuCount; ++i) {
    cpu_workers_.push_back(std::make_unique<CpuWorker>(i, *this));
    cpu_workers_.back()->Start();
  }

  std::cout << "Scheduler started with " << config.cpuCount << " cores."
            << std::endl;

  // debuggg
  // check_config_scheduler_for_discrepancies(config);

  dispatch_thread_ = std::thread(&Scheduler::dispatch, this);
  global_clock_thread_ = std::thread(&Scheduler::global_clock, this);
}

void Scheduler::stop() {
  running_ = false;
  ready_queue_.shutdown();

  clock_cv_.notify_all();

  if(dispatch_thread_.joinable()){
    dispatch_thread_.join();
  }
  for (auto& worker : cpu_workers_) {
      worker->Stop();
      worker->Join();
  }

  cpu_workers_.clear();
  
  if(global_clock_thread_.joinable()){
    global_clock_thread_.join();
  }

  memory_manager_.reset();

  std::cout << "Scheduler stopped." << std::endl;
  std::cout << "Number of cycles from this run: " << ticks_.load() << std::endl;
}

void Scheduler::submit_process(std::shared_ptr<PCB> pcb) {
  // Initialize page table for this process if memory manager is available
  if (memory_manager_) {
    // Calculate number of pages needed for process memory
    size_t process_memory_size = pcb->heap_memory.size();
    uint32_t num_pages = (process_memory_size + memPerFrame - 1) / memPerFrame; // Round up
    memory_manager_->initialize_page_table(pcb->processID, num_pages);
  }
  
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    all_processes_map_[pcb->processName] = pcb;
  }
  ready_queue_.push(std::move(pcb));
}

void Scheduler::print_status() const {
  double cpu_utilization;
  size_t total_cores = core_count_;
  size_t cores_used = 0;
  for (const auto& worker : cpu_workers_) {
    if (!worker->IsIdle()) {
      ++cores_used;
    }
  }
  calculate_cpu_utilization(total_cores, cores_used, cpu_utilization);

  std::cout << "CPU utilization: " << static_cast<int>(cpu_utilization) << "%\n";
  std::cout << "Cores used: " << cores_used << "\n";
  std::cout << "Cores available: " << (core_count_ - cores_used) << "\n\n";
  memory_manager_->generate_memory_report(std::cout);
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
  running_processes_.push_back(std::move(pcb));
}

void Scheduler::move_to_finished(std::shared_ptr<PCB> pcb) {
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    std::lock_guard<std::mutex> lock2(finished_mutex_);
    std::erase_if(running_processes_,
                  [&](const auto& p) { return p.get() == pcb.get(); });
    finished_processes_.push_back(std::move(pcb));
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

void Scheduler::start_batch_generation(const Config& config) {
  if (batch_generating_.load()) {
    std::cout << "Batch process generation is already running." << std::endl;
    return;
  }
  
  batch_generating_ = true;
  batch_generator_thread_ = std::make_unique<std::thread>([this, config]() {
    uint32_t last_tick_seen = 0;
    
    while (batch_generating_.load()) {      
      if (get_ticks() - last_tick_seen >= batch_process_freq_) {
        last_tick_seen = get_ticks();
       std::string process_name;
       
       {
         std::lock_guard<std::mutex> lock(process_counter_mutex_);

         do {
           ++process_counter_;
           std::stringstream ss;
           ss << "p" << std::setw(2) << std::setfill('0') << process_counter_;
           process_name = ss.str();
         } while (find_process_by_name(process_name) != nullptr);
       } 

        // Generate a random memory size for the process
        std::random_device rd;                         
        std::mt19937 gen(rd());                        
        std::uniform_int_distribution<> dist(minMemPerProc, maxMemPerProc); 

        size_t memory_size = dist(gen);
       
        auto instructions = instruction_generator_.generateRandomProgram(
          config.minInstructions, 
          config.maxInstructions, 
          process_name, 
          config.min_mem_per_proc,
          config.max_mem_per_proc
        );
       
        // Check if the process can fit in the memory size allocated
        if (instructions.size() + 64 > memory_size) {
          std::cerr << "Error: Process " << process_name
                    << " has too many instructions for the allocated memory size."
                    << std::endl;  
        } else {
          auto pcb = std::make_shared<PCB>(process_name, instructions, memory_size, memory_manager_.get());
          submit_process(pcb);
        }
      }
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

void Scheduler::generate_report(const std::string& filename) const {
  std::ofstream report_file(filename);
  
  if (!report_file.is_open()) {
    return;
  }

  double cpu_utilization;
  size_t total_cores = core_count_;
  size_t cores_used = 0;
  for (const auto& worker : cpu_workers_) {
    if (!worker->IsIdle()) {
      ++cores_used;
    }
  }

  calculate_cpu_utilization(total_cores, cores_used, cpu_utilization);

  report_file << "CPU utilization: " << static_cast<int>(cpu_utilization) << "%\n";
  report_file << "Cores used: " << cores_used << "\n";
  report_file << "Cores available: " << (core_count_ - cores_used) << "\n\n";

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

void Scheduler::print_process_smi() const {
  std::cout << "     _-----_\n";
  std::cout << "    |       |\n";
  std::cout << "    |--(o)--|\n";
  std::cout << "   `---------´\n";
  std::cout << "    ( CSOPESY )\n";
  std::cout << "     `-------´\n";
  std::cout << "      ___\n";
  std::cout << "     /   \\\n";
  std::cout << "|--------------------------------------------------|\n";
  std::cout << "| PROCESS-SMI V01.00 Driver Version: 01.00         |\n";
  std::cout << "|--------------------------------------------------|\n";
  
  // Calculate CPU utilization
  double cpu_utilization;
  size_t total_cores = core_count_;
  size_t cores_used = 0;
  for (const auto& worker : cpu_workers_) {
    if (!worker->IsIdle()) {
      ++cores_used;
    }
  }
  calculate_cpu_utilization(total_cores, cores_used, cpu_utilization);
  
  std::cout << "CPU-Util: " << static_cast<int>(cpu_utilization) << "%\n";
  
  if (memory_manager_) {
    uint32_t total_mem = memory_manager_->get_total_memory();
    uint32_t used_mem = memory_manager_->get_used_memory();
    
    std::cout << "Memory Usage: " << used_mem << " / " << total_mem << "\n";
    if (total_mem > 0) {
      std::cout << "Memory Util: " << static_cast<int>((double)used_mem / total_mem * 100) << "%\n";
    } else {
      std::cout << "Memory Util: 0%\n";
    }
  } else {
    std::cout << "Memory Usage: 0 / 0\n";
    std::cout << "Memory Util: 0%\n";
  }
  
  std::cout << "|--------------------------------------------------|\n";
  std::cout << "Running processes and memory usage:\n";
  std::cout << "|--------------------------------------------------|\n";
  
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    for (const auto& pcb : running_processes_) {
      // For simplicity, assuming each process uses mem_per_proc_ memory
      std::cout << pcb->processName << " " << mem_per_proc_ / 1024 << "MiB\n";
    }
  }
  
  std::cout << "|--------------------------------------------------|\n";
}

void Scheduler::print_vmstat() const {
  if (!memory_manager_) {
    std::cout << "Memory manager not available.\n";
    return;
  }
  
  uint32_t total_memory = memory_manager_->get_total_memory();
  uint32_t used_memory = memory_manager_->get_used_memory();
  uint32_t free_memory = memory_manager_->get_free_memory();
  
  size_t total_cpu_ticks = idle_cpu_ticks_.load() + active_cpu_ticks_.load();
  
  // Convert bytes to KB for display (divide by 1024)
  uint32_t total_kb = total_memory / 1024;
  uint32_t used_kb = used_memory / 1024;
  uint32_t free_kb = free_memory / 1024;
  
  std::cout << std::setw(12) << total_kb << " K total memory\n";
  std::cout << std::setw(12) << used_kb << " K used memory\n";
  std::cout << std::setw(12) << used_kb << " K active memory\n";
  std::cout << std::setw(12) << (total_kb - used_kb) << " K inactive memory\n";
  std::cout << std::setw(12) << free_kb << " K free memory\n";
  std::cout << std::setw(12) << 0 << " K buffer memory\n";
  std::cout << std::setw(12) << 0 << " K swap cache\n";
  std::cout << std::setw(12) << 0 << " K total swap\n";
  std::cout << std::setw(12) << 0 << " K used swap\n";
  std::cout << std::setw(12) << 0 << " K free swap\n";
  std::cout << std::setw(12) << active_cpu_ticks_.load() << " non-nice user cpu ticks\n";
  std::cout << std::setw(12) << 0 << " nice user cpu ticks\n";
  std::cout << std::setw(12) << active_cpu_ticks_.load() << " system cpu ticks\n";
  std::cout << std::setw(12) << idle_cpu_ticks_.load() << " idle cpu ticks\n";
  std::cout << std::setw(12) << 0 << " IO-wait cpu ticks\n";
  std::cout << std::setw(12) << 0 << " IRQ cpu ticks\n";
  std::cout << std::setw(12) << 0 << " softirq cpu ticks\n";
  std::cout << std::setw(12) << 0 << " stolen cpu ticks\n";
  std::cout << std::setw(12) << memory_manager_->get_pages_paged_in() << " pages paged in\n";
  std::cout << std::setw(12) << memory_manager_->get_pages_paged_out() << " pages paged out\n";
  std::cout << std::setw(12) << 0 << " pages swapped in\n";
  std::cout << std::setw(12) << 0 << " pages swapped out\n";
  std::cout << std::setw(12) << 0 << " interrupts\n";
  std::cout << std::setw(12) << 0 << " CPU context switches\n";
  std::cout << std::setw(12) << std::time(nullptr) << " boot time\n";
  
  size_t total_processes = 0;
  {
    std::lock_guard<std::mutex> lock(running_mutex_);
    total_processes += running_processes_.size();
  }
  {
    std::lock_guard<std::mutex> lock(finished_mutex_);
    total_processes += finished_processes_.size();
  }
  
  std::cout << std::setw(12) << total_processes << " forks\n";
}

}