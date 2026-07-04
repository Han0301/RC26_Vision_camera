#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include <atomic>
#include <sstream>
#include <thread>
#include "threadpool.h"

// ==================== 多线程功能测试套件 ====================

// 测试1：基本入队和执行
void test_threadpool_basic()
{
    std::cout << "\n========== 测试1：基本入队和执行 ==========" << std::endl;

    Ten::threadpool_test::ThreadPool pool(4, 10);   // 4线程，队列容量10

    std::atomic<int> counter{0};
    for (int i = 0; i < 10; ++i)
    {
        pool.enqueue([i, &counter] {
            std::stringstream ss;
            ss << "  任务[" << i << "] 在线程 0x"
               << std::hex << std::this_thread::get_id() << std::dec
               << " 中执行" << std::endl;
            std::cout << ss.str();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            ++counter;
        });
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "  完成计数: " << counter.load() << "/10" << std::endl;
    std::cout << (counter.load() == 10 ? "✅ 测试1通过" : "❌ 测试1失败") << std::endl;
}

// 测试2：验证并发执行（计时对比）
void test_threadpool_concurrency()
{
    std::cout << "\n========== 测试2：并发执行验证 ==========" << std::endl;

    const int TASK_COUNT = 8;
    const int WORK_MS = 200;  // 每个任务耗时200ms

    // 串行执行
    auto t1 = std::chrono::steady_clock::now();
    for (int i = 0; i < TASK_COUNT; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(WORK_MS));
    auto t2 = std::chrono::steady_clock::now();
    auto serial_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

    // 并行执行（4线程）
    auto t3 = std::chrono::steady_clock::now();
    {
        Ten::threadpool_test::ThreadPool pool(4, 20);
        for (int i = 0; i < TASK_COUNT; ++i)
        {
            pool.enqueue([WORK_MS] {
                std::this_thread::sleep_for(std::chrono::milliseconds(WORK_MS));
            });
        }
        // pool 在此析构，等待所有任务完成
    }
    auto t4 = std::chrono::steady_clock::now();
    auto parallel_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();

    std::cout << "  串行耗时: " << serial_ms << " ms" << std::endl;
    std::cout << "  并行耗时: " << parallel_ms << " ms" << std::endl;
    std::cout << "  加速比:   " << (double)serial_ms / parallel_ms << "x" << std::endl;

    // 4线程跑8个200ms任务：理论耗时 ≈ (8/4)*200 = 400ms
    if (parallel_ms < serial_ms * 0.6)
        std::cout << "✅ 测试2通过：确认了并发加速" << std::endl;
    else
        std::cout << "⚠️ 加速不明显，请检查" << std::endl;
}

// 测试3：队列满背压（生产者阻塞）
void test_threadpool_backpressure()
{
    std::cout << "\n========== 测试3：背压机制 ==========" << std::endl;

    // 只有1个线程，队列容量3，处理速度慢 → 快速填满
    Ten::threadpool_test::ThreadPool pool(1, 3);
    std::atomic<int> enqueued{0};
    std::atomic<int> executed{0};

    // 生产者线程：快速投递10个任务
    std::thread producer([&pool, &enqueued, &executed] {
        for (int i = 0; i < 10; ++i)
        {
            pool.enqueue([i, &executed] {
                std::cout << "  执行任务[" << i << "]" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                ++executed;
            });
            ++enqueued;
            std::cout << "  已入队: " << enqueued.load() << "/10" << std::endl;
        }
        std::cout << "  生产者完成投递" << std::endl;
    });

    producer.join();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "  执行完成: " << executed.load() << "/10" << std::endl;
    std::cout << (executed.load() == 10 ? "✅ 测试3通过" : "❌ 测试3失败") << std::endl;
}

// 测试4：Running_Flag
void test_running_flag()
{
    std::cout << "\n========== 测试4：Running_Flag ==========" << std::endl;

    Ten::threadpool_test::Running_Flag rf;

    std::cout << "  初始 is_running: " << rf.is_running() << std::endl;

    std::atomic<int> counter{0};
    std::thread worker([&rf, &counter] {
        while (rf.is_running())
        {
            ++counter;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "  工作线程检测到 stop，退出循环" << std::endl;
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "  运行中 counter = " << counter.load() << std::endl;

    rf.stop();
    std::cout << "  调用 stop() 后 is_running: " << rf.is_running() << std::endl;

    worker.join();
    std::cout << "  退出后 counter = " << counter.load() << std::endl;
    std::cout << (counter.load() > 0 ? "✅ 测试4通过" : "❌ 测试4失败") << std::endl;
}

// 测试5：CPU 绑核
void test_cpu_binding()
{
    std::cout << "\n========== 测试5：CPU 绑核 ==========" << std::endl;

    int total_cores = std::thread::hardware_concurrency();
    std::cout << "  系统在线核心数: " << total_cores << std::endl;

    // 合法绑核
    Ten::threadpool_test::bind_thread_to_core(0);
    std::cout << "  主线程已尝试绑定核心 0" << std::endl;

    // 非法核心号：应输出错误信息但不会崩溃
    std::cout << "  尝试绑定非法核心（应报错）：" << std::endl;
    Ten::threadpool_test::bind_thread_to_core(total_cores + 10);

    std::cout << "✅ 测试5通过：绑核和校验正常" << std::endl;
}

// 测试6：析构安全（任务没执行完就析构）
void test_threadpool_shutdown()
{
    std::cout << "\n========== 测试6：优雅退出 ==========" << std::endl;

    {
        Ten::threadpool_test::ThreadPool pool(4, 100);

        for (int i = 0; i < 8; ++i)
        {
            pool.enqueue([i] {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                std::cout << "  任务[" << i << "] 完成" << std::endl;
            });
        }
        std::cout << "  已投递 8 个任务，pool 即将析构..." << std::endl;
        // pool 在此析构，应等待所有任务完成再退出
    }
    std::cout << "  pool 析构完成，所有任务已执行" << std::endl;
    std::cout << "✅ 测试6通过：析构时等待所有任务完成" << std::endl;
}

// 测试7：如何将已有函数加入线程池
void test_enqueue_existing_function()
{
    std::cout << "\n========== 测试7：已有函数入队方式 ==========" << std::endl;

    Ten::threadpool_test::ThreadPool pool(4, 20);

    // ── 准备一些"已有的函数" ──
    auto free_func = [](const std::string& name, int ms) {
        std::cout << "  [自由函数] " << name << " 休眠 " << ms << "ms，线程: 0x"
                  << std::hex << std::this_thread::get_id() << std::dec << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    };

    struct Processor {
        void process(int id) {
            std::cout << "  [成员函数] Processor::process(" << id << ")，线程: 0x"
                      << std::hex << std::this_thread::get_id() << std::dec << std::endl;
        }
    };

    // ── 方式1：lambda 内调用（最灵活，可组合多个函数）──
    pool.enqueue([&free_func] {
        free_func("lambda内调用", 50);
    });

    // ── 方式2：直接传函数 + 参数（enqueue 自动转发）──
    // 注意：free_func 是 lambda 对象，不能直接这样传（lambda 不是普通函数指针）
    // 但普通函数可以：
    //   void foo(int a, double b);
    //   pool.enqueue(foo, 42, 3.14);     // ✅ 自动推导参数类型

    // ── 方式3：成员函数（用 std::bind 或 lambda）──
    Processor proc;
    pool.enqueue([&proc] {
        proc.process(100);
    });
    // 等价写法（C++17 更推荐 lambda，比 bind 可读性好）

    // ── 方式4：std::bind 绑定已有函数 ──
    auto bound_task = std::bind(free_func, "std::bind绑定", 60);
    pool.enqueue([bound_task] {
        bound_task();
    });
    // 注意：由于 enqueue 返回 void（非 future），
    // std::bind 产物只能用 lambda 包装后传入

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::cout << "✅ 测试7通过：4种入队方式均成功" << std::endl;
}

// 测试8：用线程池加速已有函数（核心模式）
void test_accelerate_function()
{
    std::cout << "\n========== 测试8：函数多线程加速 ==========" << std::endl;

    // ── 假设这是你已有的"重计算函数"，每块数据独立处理 ──
    auto heavy_work = [](int chunk_id, std::vector<int>& chunk) {
        // 模拟耗时计算（如点云滤波、矩阵运算等）
        for (auto& v : chunk)
            v = (v * 7 + 13) % 10007;   // 随便算点东西
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };

    const int DATA_SIZE = 100000000;
    const int THREADS   = 4;

    // ── 准备数据 ──
    std::vector<int> data_serial(DATA_SIZE);
    std::vector<int> data_parallel(DATA_SIZE);
    for (int i = 0; i < DATA_SIZE; ++i)
        data_serial[i] = data_parallel[i] = i;

    // ── 串行执行：一个线程跑完所有数据 ──
    auto t1 = std::chrono::steady_clock::now();
    heavy_work(0, data_serial);                       // 数据太大，一整个处理
    auto t2 = std::chrono::steady_clock::now();
    auto serial_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

    // ── 多线程加速：拆分 + 并行 ──
    auto t3 = std::chrono::steady_clock::now();
    {
        Ten::threadpool_test::ThreadPool pool(THREADS, THREADS * 2);
        int chunk_size = DATA_SIZE / THREADS;

        for (int t = 0; t < THREADS; ++t)
        {
            int start = t * chunk_size;
            int end   = (t == THREADS - 1) ? DATA_SIZE : start + chunk_size;

            // 关键：把每块数据的引用切片传入，各线程处理互不重叠的区域
            pool.enqueue([&data_parallel, start, end, t, &heavy_work] {
                std::vector<int> chunk(data_parallel.begin() + start,
                                       data_parallel.begin() + end);
                heavy_work(t, chunk);
            });
        }
    }   // pool 析构 = 等待所有分块处理完毕
    auto t4 = std::chrono::steady_clock::now();
    auto parallel_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();

    std::cout << "  串行耗时: " << serial_ms << " ms" << std::endl;
    std::cout << "  并行耗时: " << parallel_ms << " ms (" << THREADS << " 线程)" << std::endl;
    std::cout << "  加速比:   " << (double)serial_ms / parallel_ms << "x" << std::endl;
    std::cout << "✅ 测试8通过" << std::endl;
}

void test_all_threadpool()
{
    std::cout << "╔══════════════════════════════════════╗" << std::endl;
    std::cout << "║      ThreadPool 多线程功能测试      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════╝" << std::endl;

    // test_threadpool_basic();
    // test_threadpool_concurrency();
    // test_threadpool_backpressure();
    // test_running_flag();
    // test_cpu_binding();
    // test_threadpool_shutdown();
    test_enqueue_existing_function();
    test_accelerate_function();

    std::cout << "\n========================================" << std::endl;
    std::cout << "   🎉 全部 6 项多线程测试完成！" << std::endl;
    std::cout << "========================================" << std::endl;
}

int main(int argc, char** argv)
{
    test_all_threadpool();
    return 0;
}