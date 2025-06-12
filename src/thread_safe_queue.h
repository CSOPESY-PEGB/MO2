
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
    // adds items to the back of the queue
    void push() {

    }

    void wait_and_pop() {

    }
private:
    mutable std::mutex m_mutex;
    std::queue<T> m_queue;
    std::condition_variable m_cond;
};



#endif //THREAD_SAFE_QUEUE_H
