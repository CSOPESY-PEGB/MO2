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
  current_task_ = std::move(pcb);
  is_idle_ = false;
  condition_variable_.notify_one();
}

void CpuWorker::Run() {
  while (scheduler_.IsRunning()) {
    std::unique_lock<std::mutex> lock(mutex_);

    condition_variable_.wait(lock, [this] {
      return shutdown_requested_.load() || !is_idle_.load();
    });
    
    if (shutdown_requested_.load()) {
      break;
    }

    if (!current_task_) {
      is_idle_ = true;
      continue;
    }

    lock.unlock();
    ExecuteProcess(current_task_, time_quantum_);

    current_task_ = nullptr;
    is_idle_ = true;
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