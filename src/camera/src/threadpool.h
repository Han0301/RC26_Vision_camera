#ifndef __THREADPOOL_H_
#define __THREADPOOL_H_

#include <iostream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <sys/resource.h>

namespace Ten::threadpool_test
{
// 线程池类
class ThreadPool
{
public:
    // 构造函数和析构函数
    ThreadPool(size_t numThreads,size_t task_max_size);
    ~ThreadPool();
 
    // 添加任务到线程池
    template<class F, class... Args>
    void enqueue(F&& f, Args&&... args);
    
private:
    std::vector<std::thread> workers_lists_;           // 线程池
    std::queue<std::function<void()>> tasks_queue_;    // 任务队列
    size_t task_max_size_;                                // 任务队列最大长度
    std::mutex queue_mutex_;                             // 互斥锁，保护任务队列
    std::condition_variable condition_;                // 条件变量，用于线程等待和唤
    bool stop_;      // 标志位，指示整个线程池是否停止
};  // class ThreadPool

// 线程运行标志类
class Running_Flag
{
public:
    Running_Flag() : flag_(true) {}
    void stop() { flag_.store(false); }
    bool is_running() const { return flag_.load(); }
private:
    std::atomic<bool> flag_;        // 运行在多线程环境下的标志位，使用原子操作保证线程安全
};

// 将当前线程绑定到指定CPU核心
void bind_thread_to_core(int core_id);

inline ThreadPool::ThreadPool(size_t numThreads, size_t task_max_size)
     : stop_(false), 
    task_max_size_(task_max_size)
{
    for (size_t i = 0; i < numThreads; ++i)
    {
        workers_lists_.emplace_back([this] {
            while (true)
            {
                // 在线程接取任务的时候加🔓
                std::function<void()> task;     
                {
                    std::unique_lock<std::mutex> lock(this->queue_mutex_);
                    
                    // 0 线程池停止 或 任务队列不为空，则继续执行任务，否则挂起等待唤醒
                    this->condition_.wait(lock, [this] { return this->stop_ || !this->tasks_queue_.empty(); });
                    // 1 如果线程池停止 且 任务队列为空，则退出循环
                    if (this->stop_ && this->tasks_queue_.empty())     
                        return;
                    // 2 如果线程池未停止 且 任务队列不为空，则取出任务
                    task = std::move(this->tasks_queue_.front());
                    // 3 取出任务后，弹出队列
                    this->tasks_queue_.pop();
                }
                // 出🔓执行任务，不影响其他线程接任务
                task();
            }
        });
    }
}

inline ThreadPool::~ThreadPool()
{
    // 1 设置停止标志位
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        stop_ = true;
    }
    // 2 唤醒所有线程，通知它们 调wait检查 并退出
    condition_.notify_all();
    // 3 等待所有线程 完成任务并退出 或 直接退出
    for (std::thread &worker : workers_lists_)
        worker.join();
}

template<class F, class... Args>
inline void ThreadPool::enqueue(F&& f, Args&&... args)
{
    // 1 创建一个任务，将函数和参数绑定在一起
    auto task = std::bind(std::forward<F>(f), std::forward<Args>(args)...);
    // 2 将任务添加到任务队列
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        // 如果任务队列已满，则等待
        condition_.wait(lock, [this] { return tasks_queue_.size() < task_max_size_; });
        // 添加任务到队列
        tasks_queue_.emplace(std::move(task));
    }
    // 3 通知一个线程有新任务可执行
    condition_.notify_one();
}

} // namespace Ten::threadpool_test
#endif