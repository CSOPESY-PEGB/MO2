#ifndef OSEMU_THREAD_SAFE_QUEUE_H_
#define OSEMU_THREAD_SAFE_QUEUE_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>

// A thread-safe queue for holding any type T.
// This is a template class, so the implementation is in the header.
template <typename T>
class ThreadSafeQueue {
 public:
  // Adds an item to the back of the queue.
  void push(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(std::move(value));
    cond_.notify_one();
  }

  // Waits until an item is available, then removes it from the front and gives
  // it to the caller.
  void wait_and_pop(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);
    // The thread will wait() on the condition variable and go to sleep until it
    // is notified AND the condition in the lambda (the queue is not empty) is
    // true. This prevents spurious wakeups.
    cond_.wait(lock, [this] { return !queue_.empty(); });

    value = std::move(queue_.front());
    queue_.pop();
  }

 private:
  mutable std::mutex mutex_;
  std::queue<T> queue_;
  std::condition_variable cond_;
};

#endif  // OSEMU_THREAD_SAFE_QUEUE_H_
