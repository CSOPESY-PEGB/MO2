
#ifndef THREAD_SAFE_QUEUE_H
#define THREAD_SAFE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>

// thread_safe queue for holding std::shared_ptr<PCB>
// this is in the header because it's a template
template<typename T>
class ThreadSafeQueue{
public:
    // Adds an item to the back of the queue.
    void push(T value) {
        std::lock_guard<std::mutex> lock(m_mutex);         // Lock the mutex to ensure only one thread can modify the queue at a time
        m_queue.push(std::move(value));         // Add the item to the underlying std::queue.
        m_cond.notify_one();         // Notify one waiting thread that an item is now available.
    }

    // Waits until an item is available, then removes it from the front and gives it to the caller.
    void wait_and_pop(T& value) {
        std::unique_lock<std::mutex> lock(m_mutex);   // Use a unique_lock because we need to work with the condition variable.

        // thread will wait() on the condition variable and goes tto sleep until it is notified
        // AND the condition in the lambda (the queue is not empty) is true.
        // This prevents spurious wakeups - a thread wakes up from waiting on a condition variable and finds that the
        // condition is still unsatisfied.
        m_cond.wait(lock, [this]{ return !m_queue.empty(); });

        // the lock is held and we know the queue has at least one item
        value = std::move(m_queue.front());
        m_queue.pop();
    }

private:
    mutable std::mutex m_mutex;
    std::queue<T> m_queue;
    std::condition_variable m_cond;
};



#endif //THREAD_SAFE_QUEUE_H
