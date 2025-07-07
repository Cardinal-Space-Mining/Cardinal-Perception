#include <thread>
#include <iostream>
#include <vector>
#include <random>
#include <sstream>

#include "rss.hpp"

// Generates a vector of random integers
std::vector<int> generateRandomVector(size_t length, int minVal = 0, int maxVal = 100)
{
    std::vector<int> result;
    result.reserve(length);

    // Random number generator setup
    std::random_device rd;  // Seed
    std::mt19937 gen(rd()); // Mersenne Twister engine
    std::uniform_int_distribution<> dis(minVal, maxVal);

    // Fill the vector with random values
    for (size_t i = 0; i < length; ++i)
    {
        result.push_back(dis(gen));
    }

    return result;
}

int main1()
{

    PipelineSlot<std::vector<int>> pipe1;
    PipelineSlot<std::vector<int>> pipe2;
    PipelineSlot<std::vector<int>> pipe3;

    std::thread thd1{[&pipe1, &pipe2]
                     {
                         while (pipe1)
                         {
                             auto v = pipe1.pop();
                             if (v.has_value())
                             {

                                 auto &a = v.value();
                                 for (size_t i = 0; i < a.size(); i++)
                                 {
                                     a[i]++;
                                 }
                                 pipe2.emplace(std::move(v.value()));
                             }
                         }
                     }};

    std::thread thd2{[&pipe2, &pipe3]
                     {
                         while (pipe2)
                         {
                             auto v = pipe2.pop();

                             if (v.has_value())
                             {
                                 auto &a = v.value();
                                 for (size_t i = 0; i < a.size(); i++)
                                 {
                                     a[i] /= 2;
                                 }
                                 pipe3.emplace(std::move(v.value()));
                             }
                         }
                     }};

    std::stringstream ss;
    std::thread thd3{[&pipe3, &ss]
                     {
                         while (pipe3)
                         {
                             auto v = pipe3.pop();
                             if (v.has_value())
                             {
                                 ss << v.value()[0] << '\n';
                             }
                         }
                     }};

    pipe1.emplace({1, 2, 3});
    for (size_t i = 0; i < 10000; i++)
    {
        pipe1.emplace(generateRandomVector(30, 32, 333));
    }

    pipe1.notify_exit();
    pipe2.notify_exit();
    pipe3.notify_exit();

    thd1.join();
    thd2.join();
    thd3.join();

    std::cout << ss.str();
    return EXIT_SUCCESS;
}

int main2()
{
    auto slot1 = std::make_shared<PipelineSlot<int>>();
    auto slot2 = std::make_shared<PipelineSlot<double>>();
    auto slot3 = std::make_shared<PipelineSlot<double>>();

    PipelineStep step1{slot1, slot2, [](int i)
                       { return ((double)i) + 0.5; }};

    PipelineStep step2{slot2, slot3, [](double i)
                       { return i * i * i; }};

    PipelineStep end{slot3, [](double d) -> void
                     { std::cout << d << '\n'; }};
    for (size_t i = 0; i < 20; i++)
    {
        slot1->push(i);
        std::this_thread::sleep_for(std::chrono::seconds{1});
    }
    return EXIT_SUCCESS;
}

int main3()
{
    auto pipeline = make_pipeline([](int i)
                                  { return i + 0.5; },
                                  [](double i)
                                  { return i * i * i; },
                                  [](double d)
                                  { std::cout << d << '\n'; });

    for (size_t i = 0; i < 20; i++)
    {
        pipeline.push(i);
        std::this_thread::sleep_for(std::chrono::seconds{1});
    }
    return EXIT_SUCCESS;
}

std::atomic<size_t> alloc_counter{0};

void *operator new(std::size_t count)
{
    alloc_counter++;
    return malloc(count);
}

int main()
{
    auto pipeline = make_pipeline([](int i) -> std::vector<int>
                                  { return std::vector<int>{i}; },
                                  [](std::vector<int> i) -> std::vector<int>
                                  { i[0] *= 3; return i; },
                                  [](std::vector<int> d)
                                  { std::cout << d[0] << '\n'; });
    std::cout << "A: # Allocs: " << alloc_counter << '\n';

    for (size_t i = 0; i < 20; i++)
    {
        pipeline.push(i);
        std::this_thread::sleep_for(std::chrono::seconds{1});
    }

    std::cout << "B: # Allocs: " << alloc_counter << '\n';

    return EXIT_SUCCESS;
}

int main5()
{

    auto slot1 = std::make_shared<PipelineSlot<int>>();
    auto slot2 = std::make_shared<PipelineSlot<std::vector<int>>>();
    auto slot3 = std::make_shared<PipelineSlot<std::vector<int>>>();

    PipelineStep step1{slot1, slot2, [](int i) -> std::vector<int>
                       { return std::vector<int>{i}; }};

    PipelineStep step2{slot2, slot3, [](std::vector<int> i) -> std::vector<int>
                       { i[0] *= 3; return i; }};

    PipelineStep end{slot3, [](std::vector<int> d)
                     { std::cout << d[0] << '\n'; }};

    std::cout << "A: # Allocs: " << alloc_counter << '\n';

    for (size_t i = 0; i < 20; i++)
    {
        slot1->push(i);
        std::this_thread::sleep_for(std::chrono::seconds{1});
        std::cout << "B: # Allocs: " << alloc_counter << '\n';
    }

    return EXIT_SUCCESS;
}

class MoveOnlyInt
{
public:
    MoveOnlyInt() = default;

    MoveOnlyInt(int i) : value{i}
    {
    }

    MoveOnlyInt(MoveOnlyInt &) = delete;
    MoveOnlyInt &operator=(MoveOnlyInt &) = delete;

    MoveOnlyInt(MoveOnlyInt &&other) : value(other.value) {}
    MoveOnlyInt &operator=(MoveOnlyInt &&other)
    {
        this->value = other.value;
        return *this;
    }

    operator int()
    {
        return value;
    }

private:
    int value;
};

int main6()
{
    auto slot1 = std::make_shared<PipelineSlot<MoveOnlyInt>>();
    auto slot2 = std::make_shared<PipelineSlot<MoveOnlyInt>>();
    auto slot3 = std::make_shared<PipelineSlot<MoveOnlyInt>>();

    PipelineStep step1{slot1, slot2, [](MoveOnlyInt i)
                       { return MoveOnlyInt{i + 2}; }};

    PipelineStep step2{slot2, slot3, [](MoveOnlyInt i)
                       { return MoveOnlyInt{i * 3}; }};

    PipelineStep end{slot3, [](MoveOnlyInt d)
                     { std::cout << static_cast<int>(d) << '\n'; }};

    std::cout << "A: # Allocs: " << alloc_counter << '\n';

    for (size_t i = 0; i < 20; i++)
    {
        slot1->push(i);
        std::this_thread::sleep_for(std::chrono::seconds{1});
        std::cout << "B: # Allocs: " << alloc_counter << '\n';
    }

    return EXIT_SUCCESS;
}

// Suposed to not compile
// class ImmovableNoCopyInt
// {
// public:
//     ImmovableNoCopyInt() = default;

//     ImmovableNoCopyInt(int i) : value{i}
//     {
//     }
//     operator int() const
//     {
//         return value;
//     }

//     ImmovableNoCopyInt(ImmovableNoCopyInt &) = delete;
//     ImmovableNoCopyInt &operator=(ImmovableNoCopyInt &) = delete;

//     ImmovableNoCopyInt(ImmovableNoCopyInt &&other) = delete;
//     ImmovableNoCopyInt &operator=(ImmovableNoCopyInt &&other) = delete;

// private:
//     int value;
// };

// int main7()
// {
//     auto pipeline = make_pipeline([](int i) -> ImmovableNoCopyInt
//                                   { return ImmovableNoCopyInt{i}; },
//                                   [](ImmovableNoCopyInt i) -> ImmovableNoCopyInt
//                                   { auto b = int(i) * 3; return b; },
//                                   [](ImmovableNoCopyInt d)
//                                   { std::cout << int(d) << '\n'; });
//     std::cout << "A: # Allocs: " << alloc_counter << '\n';

//     for (size_t i = 0; i < 20; i++)
//     {
//         pipeline.push(i);
//         std::this_thread::sleep_for(std::chrono::seconds{1});
//         std::cout << "B: # Allocs: " << alloc_counter << '\n';
//     }

//     return EXIT_SUCCESS;
// }

// Suposed to  compile
class ImmovableInt
{
public:
    ImmovableInt() = default;

    ImmovableInt(int i) : value{i}
    {
    }
    operator int() const
    {
        return value;
    }

    ImmovableInt(const ImmovableInt &) = default;
    ImmovableInt &operator=(const ImmovableInt &) = default;

    ImmovableInt(ImmovableInt &&other) = default;
    ImmovableInt &operator=(ImmovableInt &&other) = delete;

private:
    int value;
};

int main7()
{
    auto pipeline = make_pipeline([](int i) -> ImmovableInt
                                  { return ImmovableInt{i}; },
                                  [](ImmovableInt i) -> ImmovableInt
                                  { auto b = int(i) * 3; return b; },
                                  [](ImmovableInt d)
                                  { std::cout << int(d) << '\n'; });
    std::cout << "A: # Allocs: " << alloc_counter << '\n';

    for (size_t i = 0; i < 20; i++)
    {
        pipeline.push(i);
        std::this_thread::sleep_for(std::chrono::seconds{1});
        std::cout << "B: # Allocs: " << alloc_counter << '\n';
    }

    return EXIT_SUCCESS;
}