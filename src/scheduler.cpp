#include "scheduler.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <random>
#include "config.hpp"
#include "process_control_block.hpp"
#include <atomic>

namespace osemu {

class Scheduler::CPUWorker {
  public:
    CPUWorker(int core_id, Scheduler& scheduler)
      : core_id_(core_id), scheduler_(scheduler) {}

    void start() { thread_ = std::thread(&CPUWorker::run, this); }
    
    void join(){
      if(thread_.joinable()){
        thread_.join();
      }
    }
    
    void stop(){
      shutdown_requested_ = true;
      cv_.notify_one();
    }
    
    void assign_task(std::shared_ptr<PCB> pcb, int time_quantum){
      std::lock_guard<std::mutex> lock(mutex_);
      time_quantum_ = time_quantum;
      current_task_ = std::move(pcb);
      idle_ = false;
      cv_.notify_one(); 
    };

    bool is_idle() const { return idle_.load(); };

  private:
    void run(){
      while(scheduler_.running_.load()){
        std::unique_lock<std::mutex> lock(mutex_);

        cv_.wait(lock, [this] {
          return shutdown_requested_.load() || !idle_.load();
        });
        
        if (shutdown_requested_.load()){
          break;
        }

        if (!current_task_) {
          idle_ = true;
          continue;
        }

        lock.unlock();
        execute_process(current_task_, time_quantum_);

        current_task_ = nullptr;
        idle_ = true;
      }
    }

    void execute_process(std::shared_ptr<PCB> pcb, int tq) {
      pcb->assignedCore = core_id_;
      scheduler_.move_to_running(pcb);
      size_t first_tick = scheduler_.get_ticks();
      size_t last_tick = first_tick; 
      int steps = 0;
      
      while((tq == -1 || last_tick - first_tick < tq) && !pcb->isComplete()){
        if(!scheduler_.running_.load() || shutdown_requested_.load()){
          break;
        }

        {
            std::unique_lock<std::mutex> lock(scheduler_.clock_mutex_); 
            
            scheduler_.clock_cv_.wait(lock, [&](){
                  return scheduler_.get_ticks() > last_tick || !scheduler_.running_.load() || shutdown_requested_.load();
            });
        }
        
        if(!scheduler_.running_.load() || shutdown_requested_.load()) break;
    
        if(scheduler_.get_ticks() - last_tick >= scheduler_.delay_per_exec_){
          pcb->step(); 
          steps++;
        }
        last_tick = scheduler_.get_ticks();
      }

      if(pcb->isComplete()){
          pcb->finishTime = std::chrono::system_clock::now();
          if (scheduler_.memory_manager_) {
              scheduler_.memory_manager_->free(pcb->processID);
          }
          scheduler_.move_to_finished(pcb);
        } else {
          scheduler_.move_to_ready(pcb);
        }
    }

    int core_id_;
    std::thread thread_;
    Scheduler& scheduler_;
    std::atomic<bool> idle_{true};
    std::atomic<bool> shutdown_requested_{false};
    std::shared_ptr<PCB> current_task_;
    int time_quantum_;
    std::mutex mutex_;
    std::condition_variable cv_;
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
        if (worker->is_idle()){
          
          if (algorithm_ == SchedulingAlgorithm::FCFS) {
            int remaining_instructions = -1;
            worker->assign_task(process, remaining_instructions);
            dispatched = true;
            break;
          } else if (algorithm_ == SchedulingAlgorithm::RoundRobin) {
            int remaining_instructions = process->totalInstructions - process->currentInstruction;
            int steps_to_run = std::min((int)quantum_cycles_, remaining_instructions);
            worker->assign_task(process, steps_to_run);
            dispatched = true;
            break;
          }
        }
      }
    }
  }
}

void Scheduler::global_clock(){
  while(running_.load()){
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if(!running_.load()) break;
    
    {
        std::lock_guard<std::mutex> lock(clock_mutex_);
        ticks_++;
    }
    clock_cv_.notify_all();

    if (running_.load() && memory_manager_ && (ticks_.load() % quantum_cycles_) == 0 && ticks_.load() > 0) {
        quantum_report_counter_++;
    }
  }
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

  memory_manager_ = std::make_unique<MemoryManager>(config.max_overall_mem);
  mem_per_proc_ = config.min_mem_per_proc;

  for (uint32_t i = 0; i < config.cpuCount; ++i) {
    cpu_workers_.push_back(std::make_unique<CPUWorker>(i, *this));
    cpu_workers_.back()->start();
  }

  std::cout << "Scheduler started with " << config.cpuCount << " cores."
            << std::endl;

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
      worker->stop();
      worker->join();
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
    if (!worker->is_idle()) {
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
          auto pcb = std::make_shared<PCB>(process_name, instructions, memory_size);
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

  size_t total_cores = core_count_;
  size_t cores_used = active_cores_;
  double cpu_utilization;
  calculate_cpu_utilization(total_cores, cores_used, cpu_utilization);

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

}