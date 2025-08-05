#include "cpu_worker.h"

#include <chrono>
#include <memory>

#include "scheduler.hpp"

namespace osemu {

CpuWorker::CpuWorker(int core_id, Scheduler& scheduler)
    : core_id_(core_id), scheduler_(scheduler) {}

void CpuWorker::Start() {
  thread_ = std::thread(&CpuWorker::Run, this);
}

void CpuWorker::Stop() {
  shutdown_requested_ = true;
  condition_variable_.notify_one();
}

void CpuWorker::Join() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

void CpuWorker::AssignTask(std::shared_ptr<PCB> pcb, int time_quantum) {
  std::lock_guard<std::mutex> lock(mutex_);
  time_quantum_ = time_quantum;
  ticks_with_current_task_ = 0;
  current_task_ = std::move(pcb);
  is_idle_ = false;
  condition_variable_.notify_one();
}

void CpuWorker::Run() {
  while (scheduler_.IsRunning()) {
    // Participate in barrier synchronization
    const bool tick_ready_phase = scheduler_.tick_ready_.load();
    std::unique_lock<std::mutex> barrier_lock(scheduler_.barrier_mutex_);
    
    // Wait for the Scheduler's "go" signal for this tick
    scheduler_.dispatch_go_cv_.wait(barrier_lock, [&]() {
        return scheduler_.tick_ready_ == tick_ready_phase || shutdown_requested_.load() || !scheduler_.IsRunning();
    });

    if (shutdown_requested_.load() || !scheduler_.IsRunning()) {
        break;
    }
    
    barrier_lock.unlock();
    
    // Execute one step if we have a task
    std::unique_lock<std::mutex> self_lock(mutex_);
    if (!is_idle_.load() && current_task_) {
      // Check if time quantum expired (for round-robin scheduling)
      if (time_quantum_ > 0 && ticks_with_current_task_ >= time_quantum_) {
        // Time quantum expired - preempt the process
        scheduler_.move_to_ready(current_task_);
        current_task_ = nullptr;
        is_idle_ = true;
        ticks_with_current_task_ = 0;
      } else {
        // Execute one instruction step
        InstructionExecutionInfo info = current_task_->step();
        ticks_with_current_task_++;
        
        if (info.result == InstructionResult::PROCESS_COMPLETE) {
          // Process completed
          current_task_->finishTime = std::chrono::system_clock::now();
          if (scheduler_.memory_manager_) {
            scheduler_.memory_manager_->free(current_task_->processID);
          }
          scheduler_.move_to_finished(current_task_);
          current_task_ = nullptr;
          is_idle_ = true;
          ticks_with_current_task_ = 0;
        } else if (info.result == InstructionResult::PAGE_FAULT) {
          // Handle page fault - process will be handled by memory manager
        }
        // For SUCCESS, continue executing
      }
    }
    self_lock.unlock();
    
    // Signal completion of this tick
    {
      std::unique_lock<std::mutex> signal_lock(scheduler_.barrier_mutex_);
      scheduler_.workers_completed_step_count_.fetch_add(1);
      scheduler_.workers_done_cv_.notify_one();
    } // Release the lock here

    // Wait for next tick signal (without holding barrier_mutex_)
    {
      std::unique_lock<std::mutex> wait_lock(scheduler_.barrier_mutex_);
      scheduler_.dispatch_go_cv_.wait(wait_lock, [&]() {
        return tick_ready_phase != scheduler_.tick_ready_.load() || shutdown_requested_.load() || !scheduler_.IsRunning();
      });
    }
  }
}

void CpuWorker::ExecuteProcess(std::shared_ptr<PCB> pcb, int time_quantum) {
  pcb->assignedCore = core_id_;
  scheduler_.move_to_running(pcb);
  
  size_t first_tick = scheduler_.get_ticks();
  size_t last_tick = first_tick; 
  int steps = 0;
  
  while ((time_quantum == -1 || last_tick - first_tick < time_quantum) && 
         !pcb->isComplete()) {
    if (!scheduler_.IsRunning() || shutdown_requested_.load()) {
      break;
    }

    {
      std::unique_lock<std::mutex> lock(scheduler_.GetClockMutex()); 
      
      scheduler_.GetClockCondition().wait(lock, [&]() {
        return scheduler_.get_ticks() > last_tick || 
               !scheduler_.IsRunning() || 
               shutdown_requested_.load();
      });
    }
    
    if (!scheduler_.IsRunning() || shutdown_requested_.load()) {
      break;
    }

    if (scheduler_.get_ticks() - last_tick >= scheduler_.GetDelayPerExecution()) {
      pcb->step(); 
      steps++;
    }
    last_tick = scheduler_.get_ticks();
  }

  if (pcb->isComplete()) {
    pcb->finishTime = std::chrono::system_clock::now();
    if (scheduler_.memory_manager_) {
      scheduler_.memory_manager_->free(pcb->processID);
    }
    scheduler_.move_to_finished(pcb);
  } else {
    scheduler_.move_to_ready(pcb);
  }
}

}  // namespace osemu
