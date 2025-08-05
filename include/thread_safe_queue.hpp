#ifndef OSEMU_THREAD_SAFE_QUEUE_H_
#define OSEMU_THREAD_SAFE_QUEUE_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <deque>
#include <atomic>

template <typename T>
class ThreadSafeQueue {
public:
  ThreadSafeQueue() : shutdown_requested_(false) {}

  void push(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_requested_.load()) return;
    queue_.push_back(std::move(value));
    cond_.notify_one();
  }

  void push_front(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_requested_.load()) return;
    queue_.push_front(std::move(value));
    cond_.notify_one();
  }

  void shutdown(){
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_requested_ = true;
    cond_.notify_all();
  }

  bool wait_and_pop(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait(lock, [this] { return !queue_.empty() || shutdown_requested_.load(); });

    if (shutdown_requested_.load() && queue_.empty()) {
      return false;
    }

    value = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  void empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
  }

  std::deque<T> get_copy() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_;
  }

private:
  mutable std::mutex mutex_;
  std::deque<T> queue_;
  std::condition_variable cond_;
  std::atomic<bool> shutdown_requested_;
};

#endif