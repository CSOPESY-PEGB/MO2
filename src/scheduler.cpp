#include "scheduler.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>

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
    while (system_running_) {
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
      pcb->step();

      auto now = std::chrono::system_clock::now();
      log_file << std::format(
          "({:%m/%d/%Y %I:%M:%S%p}) Core:{} \"Hello world from {}!\"\n", now,
          core_id_, pcb->processName);

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

Scheduler::Scheduler() : running_(false) {}

Scheduler::~Scheduler() {
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
  std::cout << "----------------------------------------------------------------\n";
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
  std::cout << "----------------------------------------------------------------\n";
}

void Scheduler::move_to_running(std::shared_ptr<PCB> pcb) {
  std::lock_guard<std::mutex> lock(running_mutex_);
  running_processes_.push_back(std::move(pcb));
}

void Scheduler::move_to_finished(std::shared_p<PCB> pcb) {
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

}  // namespace osemu
