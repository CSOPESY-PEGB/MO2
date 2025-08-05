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

// Nested CPUWorker implementation
Scheduler::CPUWorker::CPUWorker(int core_id, Scheduler& scheduler)
    : core_id_(core_id), scheduler_(scheduler) {}

void Scheduler::CPUWorker::start() { 
  thread_ = std::thread(&Scheduler::CPUWorker::run, this); 
}

void Scheduler::CPUWorker::join(){
  if(thread_.joinable()){
    thread_.join();
  }
}

void Scheduler::CPUWorker::stop(){
  shutdown_requested_ = true;
  // Notify global barrier CVs to unblock ALL workers so they can exit gracefully.
  scheduler_.dispatch_go_cv_.notify_all();
  scheduler_.workers_done_cv_.notify_all();
}

void Scheduler::CPUWorker::assign_task(std::shared_ptr<PCB> pcb, int time_quantum){
    std::lock_guard<std::mutex> lock(mutex_); // Protect internal state
    time_quantum_assigned_ = time_quantum;
    current_task_ = std::move(pcb);
    steps_executed_on_this_core_ = 0; // Reset counters for new task
    ticks_since_last_instruction_ = 0;
    
    // Reset all status flags for the new task
    has_completed_task_ = false;
    has_page_faulted_ = false;
    needs_preemption_ = false;
    
    idle_ = false; // Mark worker as busy (holding a task)
    // No cv_.notify_one() here. The run() loop will pick up the change on the next tick's signal.
};

// `run()` method - Fully synchronous, always participating in barrier
void Scheduler::CPUWorker::run(){
    while(scheduler_.running_.load()){
        const bool tick_ready_phase = scheduler_.tick_ready_.load();
        std::unique_lock<std::mutex> barrier_lock(scheduler_.barrier_mutex_);
        // Wait for the Scheduler's "go" signal for this tick.
        scheduler_.dispatch_go_cv_.wait(barrier_lock, [&]() {
            return scheduler_.tick_ready_ == tick_ready_phase || shutdown_requested_.load() || !scheduler_.running_.load();
        });

        if (shutdown_requested_.load() || !scheduler_.running_.load()){
            break; // Exit thread if shutdown requested.
        }
        
        barrier_lock.unlock(); // Release global barrier mutex

        std::unique_lock<std::mutex> self_lock(mutex_); // Acquire worker's private mutex

        if (idle_.load()) {
            // Worker is currently idle (no task assigned). It simply passes through this tick.
        } else if (current_task_) { // Worker has a task and it's NOT I/O blocked.
            ticks_since_last_instruction_++;
            if (ticks_since_last_instruction_ >= scheduler_.delay_per_exec_) {
                ticks_since_last_instruction_ = 0; // Reset counter for next instruction

                InstructionExecutionInfo info = current_task_->step(); // Execute instruction
                last_step_info_ = info; // Store this for the Scheduler to inspect

                if (info.result == InstructionResult::PAGE_FAULT) { 
                  has_page_faulted_ = true; 
                }
                else if (info.result == InstructionResult::PROCESS_COMPLETE) { 
                  has_completed_task_ = true; 
                }
                else { // InstructionResult::SUCCESS
                    steps_executed_on_this_core_++;
                    if (scheduler_.algorithm_ == SchedulingAlgorithm::RoundRobin &&
                        time_quantum_assigned_ != -1 && 
                        steps_executed_on_this_core_ >= time_quantum_assigned_) {
                        needs_preemption_ = true;
                    }
                }
            }
        }
        
        self_lock.unlock(); // Release worker's own mutex
        {
          std::unique_lock<std::mutex> signal_lock(scheduler_.barrier_mutex_); // Re-acquire global barrier mutex
          scheduler_.workers_completed_step_count_.fetch_add(1); // Increment global counter
          scheduler_.workers_done_cv_.notify_one();              // Notify Scheduler

          scheduler_.dispatch_go_cv_.wait(signal_lock, [&]() { // Use signal_lock (holds barrier_mutex_)
            return tick_ready_phase != scheduler_.tick_ready_.load()  || shutdown_requested_.load() || !scheduler_.running_.load();
          });
        }
    }
}

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
    // This helper function frees cpus with expired time quantum or finished tasks
    bool one_free = find_idle_cpu(); 
    // Generate one process
    if(batch_generating_.load() && ticks_ % batch_process_freq_ == 0) generate_process(); 

    // If a free cpu exists try to pop and assign process to the free cpu.
    find_free_cpu_and_assign();
    // Signal all cores to run; IMPORTANT; all cores must participate even when idle
    // This function also blocks until all cores have participated 
    signal_execute(); 

    // Suspend all processes to handle page faults; not ideal in real world system 
    // but for this simulation ensures deterministic behavior and no deadlocks
    if (memory_manager_) {
      memory_manager_->handle_page_faults(); 
    }

    ticks_++;
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
  minInstructions = config.minInstructions;
  maxInstructions = config.maxInstructions;

  memory_manager_ = std::make_unique<MemoryManager>(config.max_overall_mem);
  mem_per_proc_ = config.min_mem_per_proc;

  // Create nested CPUWorkers for vmstat integration
  for (uint32_t i = 0; i < config.cpuCount; ++i) {
    vmstat_cpu_workers_.push_back(std::make_unique<CPUWorker>(i, *this));
    vmstat_cpu_workers_.back()->start();
  }

  std::cout << "Scheduler started with " << core_count_ << " cores."
            << std::endl;

  // debuggg
  // check_config_scheduler_for_discrepancies(config);

  dispatch_thread_ = std::thread(&Scheduler::dispatch, this);
}

void Scheduler::stop() {
  running_ = false;
  ready_queue_.shutdown();

  clock_cv_.notify_all();

  if(dispatch_thread_.joinable()){
    dispatch_thread_.join();
  }

  // Stop nested CPUWorkers
  for (auto& worker : vmstat_cpu_workers_) {
      worker->stop();
      worker->join();
  }
  vmstat_cpu_workers_.clear();
  
  // Stop external CpuWorkers if any exist
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

void Scheduler::signal_execute(){
  workers_completed_step_count_.store(0); // Reset barrier counter
  
  {
    std::unique_lock<std::mutex> lock(barrier_mutex_); // Acquire barrier mutex
    bool expected = tick_ready_.load();
    bool desired;
    do {
      desired = !expected;
    } while(!tick_ready_.compare_exchange_weak(expected, desired));
  }
  dispatch_go_cv_.notify_all(); // Signal all CPUWorker threads to process their tick

  std::unique_lock<std::mutex> lock(barrier_mutex_); // Acquire barrier mutex

  workers_done_cv_.wait(lock, [&]{
    bool done = workers_completed_step_count_.load() >= vmstat_cpu_workers_.size();
    bool shutting_down = !running_.load();
    return done || shutting_down;
  });
  
  // Barrier passed. All workers have processed their tick and updated their flags.
}

bool Scheduler::find_idle_cpu(){
  // Free all cores that need to be freed.
  bool flag = false; // Assume all cores are taken
  active_cores_ = 0;

  for(auto const& cpu: vmstat_cpu_workers_){
    std::unique_lock<std::mutex> worker_lock(cpu->mutex_); // Acquire worker's mutex
    if(cpu->has_completed_task_){
      std::shared_ptr<PCB> pcb = cpu->current_task_; // Read current_task_ safely
      pcb->finishTime = std::chrono::system_clock::now();
      cpu->idle_ = true;
      cpu->has_completed_task_ = false;
      flag = true;
      // Clean up functions
      move_to_finished(pcb);
    } else if(cpu->needs_preemption_){
        std::shared_ptr<PCB> pcb = cpu->current_task_; // Read current_task_ safely
        cpu->idle_ = true;
        cpu->needs_preemption_ = false;
        flag = true;
        move_to_ready(pcb); // Moves to ready_queue_ and removes from running_processes_
    } else if(cpu->is_idle()){
      flag = true;
    } else {
      active_cores_++; // Count active cores
    }
  }
  return flag; // If flag is true, either we freed a process or found idle cores
}

void Scheduler::find_free_cpu_and_assign(){
  for(auto const& cpu: vmstat_cpu_workers_){
    if(cpu->is_idle()){
      std::optional<std::shared_ptr<PCB>> try_process = ready_queue_.try_pop(); 
      if(try_process.has_value()){
        std::shared_ptr<PCB> process = try_process.value();
        process->assignedCore = cpu->core_id_;
        cpu->assign_task(process, quantum_cycles_);
        move_to_running(process);
      }
    }
  }
}

void Scheduler::generate_process() {
    std::string process_name;
    
    // Generate a unique process name using a mutex for shared counter
    { 
      std::lock_guard<std::mutex> lock(process_counter_mutex_);

      do {
        ++process_counter_;
        std::stringstream ss;
        ss << "p" << std::setw(2) << std::setfill('0') << process_counter_;
        process_name = ss.str();
      } while (find_process_by_name(process_name) != nullptr); // Ensure name is unique
    } 

    // Generate a random memory size for the process
    std::random_device rd;                         
    std::mt19937 gen(rd());                        
    // Use member variables for min/max memory per process (from Config)
    std::uniform_int_distribution<> dist(minMemPerProc, maxMemPerProc); 
    size_t memory_size = dist(gen);
   
    // Generate random instructions using captured config values
    auto instructions = instruction_generator_.generateRandomProgram(
      minInstructions, // Use the stored config values
      maxInstructions,
      process_name, 
      minMemPerProc, // These are for instruction generation context
      maxMemPerProc  // (match your existing usage in instruction_generator)
    );
   
    // Check if the process can fit in the allocated memory size
    if (instructions.size() + 64 > memory_size) {
      std::cerr << "Error: Process " << process_name
                << " has too many instructions for the allocated memory size."
                << std::endl;  
    } else {
      auto pcb = std::make_shared<PCB>(process_name, instructions, memory_size, memory_manager_.get());
      submit_process(pcb); // Submit the newly created process to the ready queue
    }
}

void Scheduler::start_batch_generation(const Config& config) {
  if (batch_generating_.load()) {
    std::cout << "Batch process generation is already running." << std::endl;
    return;
  }
  
  // Capture the relevant configuration parameters as member variables
  batch_generating_ = true; // Enable batch generation
  std::cout << "Started batch process generation." << std::endl;
}

void Scheduler::stop_batch_generation() {
  if (!batch_generating_.exchange(false)) { // Atomically set to false and check if it was already false
    return; // Batch generation was not running
  }

  // No batch_generator_thread_ to join or reset.

  // Report the total count of processes generated
  uint32_t count = 0;
  { // Lock to safely read the final process_counter_
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
  size_t cores_used = active_cores_;
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
  
  size_t idle_ticks = idle_cpu_ticks_.load();
  size_t active_ticks = active_cpu_ticks_.load();
  size_t total_ticks = idle_ticks + active_ticks;
  
  std::cout << "Total memory: " << total_memory << " bytes\n";
  std::cout << "Used memory: " << used_memory << " bytes\n";
  std::cout << "Free memory: " << free_memory << " bytes\n";
  std::cout << "Idle cpu ticks: " << idle_ticks << "\n";
  std::cout << "Active cpu ticks: " << active_ticks << "\n";
  std::cout << "Total cpu ticks: " << total_ticks << "\n";
  std::cout << "Num paged in: " << memory_manager_->get_pages_paged_in() << "\n";
  std::cout << "Num paged out: " << memory_manager_->get_pages_paged_out() << "\n";
}

}