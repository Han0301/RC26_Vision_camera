#ifndef _THREADPOOL_CPP_
#define _THREADPOOL_CPP_
#include "threadpool.h"

namespace Ten::threadpool_test
{
// 将线程绑定到指定的CPU核心, 避免线程在不同核心之间切换，提高性能
void bind_thread_to_core(int core_id)
{
    // 检查 core_id 是否在有效范围内
    if (core_id < 0 || core_id >= std::thread::hardware_concurrency())
    {
        std::cerr << "Error: Invalid core_id " << core_id << ". It must be between 0 and " 
                  << std::thread::hardware_concurrency() - 1 << "." << std::endl;
        return;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    pthread_t current_thread = pthread_self();
    int result = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0)
    {
        std::cerr << "Error setting thread affinity: " << std::strerror(result) << std::endl;
    }
    std::cout << "Thread " << std::this_thread::get_id() << " bound to core " << core_id << std::endl;
}
}   // namespace Ten::threadpool_test
#endif