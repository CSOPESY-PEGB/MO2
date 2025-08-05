#ifndef OSEMU_SCHEDULER_H_
#define OSEMU_SCHEDULER_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <unordered_map>
#include "process_control_block.hpp"
#include "thread_safe_queue.hpp"
#include "instruction_generator.hpp"
#include "config.hpp"
#include "memory_manager.hpp"
#include "cpu_worker.h"

namespace osemu {

class Config;

class Scheduler {
 public:
  Scheduler();
  ~Scheduler();

  void dispatch();
  void global_clock();
  void check_config_scheduler_for_discrepancies(const Config& config);

  void start(const Config& config);
  void stop();

  void submit_process(std::shared_ptr<PCB> pcb);
  void print_status() const;

  void start_batch_generation(const Config& config);
  void stop_batch_generation();
  void calculate_cpu_utilization(size_t& total_cores, size_t& cores_used,
                                 double& cpu_utilization) const;
  bool is_generating() const { return batch_generating_; }
  std::shared_ptr<PCB> find_process_by_name(const std::string& name) const;
  
  void move_to_running(std::shared_ptr<PCB> pcb);
  void move_to_finished(std::shared_ptr<PCB> pcb);
  void move_to_ready(std::shared_ptr<PCB> pcb);
  
  void generate_report(const std::string& filename = "csopesy-log.txt") const;
  void print_process_smi() const;
  void print_vmstat() const;

  size_t get_ticks() const { return ticks_.load(); }
  MemoryManager* get_memory_manager() const { return memory_manager_.get(); }
  
  // Public interface methods for CpuWorker access  
  std::mutex& GetClockMutex() { return clock_mutex_; }
  std::condition_variable& GetClockCondition() { return clock_cv_; }
  bool IsRunning() const { return running_.load(); }
  size_t GetDelayPerExecution() const { return delay_per_exec_; }

  std::unique_ptr<MemoryManager> memory_manager_;

  // --- Barrier Synchronization for CPU Workers ---
  std::atomic<int> workers_completed_step_count_{0}; // Count of workers done with current tick
  std::condition_variable dispatch_go_cv_; // Scheduler notifies workers to start their tick
  std::condition_variable workers_done_cv_; // Workers notify Scheduler they've finished their tick
  std::mutex barrier_mutex_; // Mutex protecting barrier counts and CVs
  
 private:
  friend class CpuWorker;
  friend class CPUWorker;
  
  // --- Nested CPUWorker Class Definition ---
  class CPUWorker {
  public:
    CPUWorker(int core_id, Scheduler& scheduler);

    void start();
    void join();
    void stop();
    
    // assign_task now only sets the task; no internal cv.notify_one related to immediate execution.
    void assign_task(std::shared_ptr<PCB> pcb, int time_quantum);

    bool is_idle() const { return idle_.load(); };

    // Public members for Scheduler to inspect/manage worker state
    std::shared_ptr<PCB> current_task_; // The PCB currently held by this core
    std::atomic<bool> idle_{true};       // True if no PCB is assigned
    
    // Atomic flags to report status back to the Scheduler for processing this tick
    std::atomic<bool> has_completed_task_{false}; 
    std::atomic<bool> has_page_faulted_{false}; 
    std::atomic<bool> needs_preemption_{false}; 
    InstructionExecutionInfo last_step_info_; // Stores info if a step was executed (e.g., VA for fault)
    
    int core_id_; // Core ID (public for reporting)

    std::mutex mutex_; // Protects `current_task_` and internal counters/flags

  private:
    std::thread thread_;
    Scheduler& scheduler_; // Reference to the parent scheduler
    std::atomic<bool> shutdown_requested_{false};

    // Internal state specific to the current task on this core
    int time_quantum_assigned_ = -1; // -1 for FCFS (run until completion), or assigned RR quantum
    size_t steps_executed_on_this_core_ = 0; // Steps run within this worker's current assignment
    size_t ticks_since_last_instruction_ = 0; // Counter for `delay_per_exec_`

    void run(); // The main loop that processes ticks
  };

  bool find_idle_cpu();
  void signal_execute();
  void find_free_cpu_and_assign();
  void generate_process();
  
  std::atomic<int> cores_ready_for_next_tick_{0};
  int total_cores_{0};

  std::atomic<bool> running_;
  std::vector<std::unique_ptr<CpuWorker>> cpu_workers_; // External CpuWorker for compatibility
  std::vector<std::unique_ptr<CPUWorker>> vmstat_cpu_workers_; // Nested CPUWorker for vmstat integration

  ThreadSafeQueue<std::shared_ptr<PCB>> ready_queue_;

  mutable std::mutex running_mutex_;
  mutable std::mutex finished_mutex_;

  mutable std::mutex map_mutex_;
  std::unordered_map<std::string, std::shared_ptr<PCB>> all_processes_map_;

  std::vector<std::shared_ptr<PCB>> running_processes_;
  std::vector<std::shared_ptr<PCB>> finished_processes_;

  std::thread dispatch_thread_;

  std::atomic<bool> batch_generating_;
  std::unique_ptr<std::thread> batch_generator_thread_;
  InstructionGenerator instruction_generator_;
  int process_counter_;
  mutable std::mutex process_counter_mutex_;
  
  std::atomic<size_t> ticks_{0}; 
  mutable std::mutex clock_mutex_; 
  std::condition_variable clock_cv_; 
  std::thread global_clock_thread_;
  std::atomic<bool> tick_ready_{false};
  
  // --- NEW and MODIFIED MEMBERS for Memory Management ---
  uint32_t mem_per_proc_{4096};
  std::atomic<size_t> quantum_report_counter_{0};
  // ---------------------------------------------------

  size_t batch_process_freq_{1};
  size_t delay_per_exec_{0};
  size_t quantum_cycles_{5};
  size_t core_count_{0};
  size_t active_cores_{0};
  
  size_t maxOverallMemory{1024};
  size_t memPerFrame{64};
  size_t minMemPerProc{512};
  size_t maxMemPerProc{1024};

  SchedulingAlgorithm algorithm_{SchedulingAlgorithm::FCFS};
  
  // Tracking for vmstat
  std::atomic<size_t> idle_cpu_ticks_{0};
  std::atomic<size_t> active_cpu_ticks_{0};
  std::atomic<size_t> pages_paged_in_{0};
  std::atomic<size_t> pages_paged_out_{0};

};

}  

#endif
