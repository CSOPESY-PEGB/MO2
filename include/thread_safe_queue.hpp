#ifndef OSEMU_THREAD_SAFE_QUEUE_H_
#define OSEMU_THREAD_SAFE_QUEUE_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <iostream>

template <typename T>
class ThreadSafeQueue {

  public:
  void push(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(std::move(value));
    cond_.notify_one();
  }
  
  void shutdown(){
    shutdown_requested_ = true;
    cond_.notify_all();
  }

  bool wait_and_pop(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait(lock, [this] { return !queue_.empty(); });
    
    if (shutdown_requested_.load() && queue_.empty()) {
      return false;
    }

    // We can safely pop the front element.
    value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  void empty() {
    std::lock_guard lock(mutex_);
    while(!queue_.empty()){
      queue_.pop();
      std::cout << "emptied queue" << std::endl;
    }
  }

 private:
  mutable std::mutex mutex_;
  std::queue<T> queue_;
  std::condition_variable cond_;
  std::atomic_bool shutdown_requested_;
};

#endif  
