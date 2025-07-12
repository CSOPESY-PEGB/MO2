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
  // non persistant
  public:
    CPUWorker(int core_id, Scheduler& scheduler)
      : core_id_(core_id), scheduler_(scheduler) {}

    
    std::shared_ptr<PCB> current_task_;

    // func assign: assign process to worker with time quantum; if FCFS time quantum is process length, if RoundRobin time quantum is quantum_cycles_
    void assign_task(std::shared_ptr<PCB> pcb, int time_quantum){
      std::lock_guard<std::mutex> lock(mutex_);
      current_task_ = std::move(pcb);
      time_quantum_ = time_quantum;
      idle_ = false;
      step = 0; // Reset step counter when a new task is assigned
    };


    // func execute 1 step: execute one step and then check if the process is complete and then move it to finished or ready queue and then is_idle is false
    void execute_process(std::shared_ptr<PCB> pcb) {
      pcb->assignedCore = core_id_;
      scheduler_.move_to_running(pcb);

      if(!scheduler_.running_.load() || shutdown_requested_.load()){
        return; // If the scheduler is not running or shutdown is requested, exit
      } else {
        // Execute a step and then check if the process is complete
        pcb->step();

        if (pcb->isComplete()) {
          pcb->finishTime = std::chrono::system_clock::now();
          scheduler_.move_to_finished(pcb);
          idle_ = true; // Mark worker as idle if process is complete
          current_task_.reset(); // Clear the current task
        } else if(step >= time_quantum_) {
          // If the process has executed for the quantum time, move it to ready queue
          scheduler_.move_to_ready(pcb);
          idle_ = true; // Mark worker as idle after moving to ready queue
          current_task_.reset(); // Clear the current task
        } else {
          // If the process is not complete and still within quantum, continue executing
          step++;
        }
      }
    }

    // func is_idle: check if the worker is idle by checking if it has a process
    bool is_idle() const { return idle_; };

    private:
      int core_id_;
      Scheduler& scheduler_;
      int time_quantum_{0};
      bool idle_{true};
      std::atomic_bool shutdown_requested_{false};
      mutable std::mutex mutex_;
      int step = 0; // Counter for the number of steps executed
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
  // EDIT Dispatch function tomorrow
  std::vector<std::unique_ptr<CPUWorker>> cpu_workers_;
  int idle_cores = 0;

  while(running_.load()){
    std::shared_ptr<PCB> process;
    // check if a CPU is idle and if so, assign a process to it
    // asign first and then step on cycle for the cpu
    // remove all mutex
    
    // Check if the number of CPU workers matches the core count and create new workers if necessary
    if (cpu_workers_.size() != core_count_) {
      cpu_workers_.clear();
      for (int i = 0; i < core_count_; ++i) {
        cpu_workers_.emplace_back(std::make_unique<CPUWorker>(i, *this));
      }
    }

    // loop through workers and assign processes if they are idle
    idle_cores = 0;
    for (const auto& worker : cpu_workers_) {
      if (worker->is_idle()) {
        idle_cores++;
        if (ready_queue_.wait_and_pop(process)) {
          int time_quantum = (algorithm_ == SchedulingAlgorithm::RoundRobin) ? quantum_cycles_ : process->totalInstructions;
          worker->assign_task(process, time_quantum);
          idle_cores--; // Decrease idle cores count since we assigned a process
        } else {
          break; // Exit if no process is available
        }
      }
    }

    // Step through each worker to execute 1 step of the assigned process
    for (const auto& worker : cpu_workers_) {
      if (!worker->is_idle()) {
        worker->execute_process(worker->current_task_);
      }
    }

    // update the number of active cores
    active_cores_ = core_count_ - idle_cores;

  }
}

void Scheduler::start(const Config& config) {
  running_ = true;
  delay_per_exec_ = config.delayCyclesPerInstruction;
  quantum_cycles_ = config.quantumCycles;
  algorithm_ = config.scheduler;
  core_count_ = config.cpuCount;

  std::cout << "Scheduler started with " << config.cpuCount << " cores."
            << std::endl;

  dispatch_thread_ = std::thread(&Scheduler::dispatch, this);
}

void Scheduler::stop() {
  running_ = false;
  ready_queue_.shutdown();

  clock_cv_.notify_all();

  if(dispatch_thread_.joinable()){
    dispatch_thread_.join();
  }

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
  size_t cores_used = active_cores_;
  calculate_cpu_utilization(total_cores, cores_used, cpu_utilization);

  std::cout << "CPU utilization: " << static_cast<int>(cpu_utilization) << "%\n";
  std::cout << "Cores used: " << cores_used << "\n";
  std::cout << "Cores available: " << (core_count_ - cores_used) << "\n\n";

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
    uint32_t cpu_cycles = 0;
    
    while (batch_generating_.load()) {
      cpu_cycles++;
      
      if (cpu_cycles % config.processGenFrequency == 0) {
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

        auto instructions = instruction_generator_.generateRandomProgram(
          config.minInstructions, 
          config.maxInstructions, 
          process_name
        );
        
        auto pcb = std::make_shared<PCB>(process_name, instructions);
        submit_process(pcb);
      }
      
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
    std::cerr << "Failed to create report file: " << filename << std::endl;
    return;
  }

  size_t total_cores;
  size_t cores_used;
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
