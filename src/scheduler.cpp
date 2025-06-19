#include "scheduler.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <random>
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
    while (true) {
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
  if (running_.load() || m_generator_running.load()) {
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
  stop_generator();

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

void Scheduler::move_to_running(std::shared_ptr<PCB> pcb) {
  std::lock_guard<std::mutex> lock(running_mutex_);
  running_processes_.push_back(std::move(pcb));
}

void Scheduler::move_to_finished(std::shared_ptr<PCB> pcb) {
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

void Scheduler::start_generator(const Config& config) {
  if (m_generator_running.load()) {
    std::cout << "Process generator is already running.\n";
    return;
  }

  m_generator_running = true;
  m_generator_thread = std::thread(&Scheduler::generator_loop, this, config);
  std::cout << "Continuous process generator started.\n";
}

void Scheduler::stop_generator() {
  if (!m_generator_running.exchange(false)) { // exchange checks and sets atomically
    return; // It was already stopped
  }

  if (m_generator_thread.joinable()) {
    m_generator_thread.join();
  }
  std::cout << "Continuous process generator stopped.\n";
}



void Scheduler::generator_loop(Config config) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distrib(config.minInstructions, config.maxInstructions);
  static std::atomic<int> process_id_counter = 1;

  while (m_generator_running.load()) {
    const int batch_size = 2;
    std::cout << "[Generator] Creating a batch of " << batch_size << " processes...\n";

    for (int i = 0; i < batch_size; ++i) {
      if (!m_generator_running.load()) break;

      int current_id = process_id_counter.fetch_add(1);
      std::string name = std::format("proc{:02}", current_id);

      size_t instructions = distrib(gen);

      auto pcb = std::make_shared<PCB>(name, instructions);

      this->submit_process(pcb);
    }

    // Use the frequency from the Config file
    std::this_thread::sleep_for(std::chrono::seconds(config.processGenFrequency));
  }
}

}  // namespace osemu
