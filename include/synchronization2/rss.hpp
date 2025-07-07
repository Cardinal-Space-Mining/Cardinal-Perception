/*******************************************************************************
 *   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
 *                                                                              *
 *                                 ;xxxxxxx:                                    *
 *                                ;$$$$$$$$$       ...::..                      *
 *                                $$$$$$$$$$x   .:::::::::::..                  *
 *                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
 *                         :$$$$$&X;      .xX:::::::::::::.::...                *
 *                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
 *                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
 *               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
 *              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
 *               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
 *                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
 *                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
 *                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
 *                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
 *                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
 *                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
 *                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
 *               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
 *              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
 *               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
 *                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
 *                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
 *               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
 *               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
 *               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
 *                .;;;;;;;;:;;    +$$$$$$$$$                                    *
 *                  .;;;;;;.       X$$$$$$$:                                    *
 *                                                                              *
 *   Unless required by applicable law or agreed to in writing, software        *
 *   distributed under the License is distributed on an "AS IS" BASIS,          *
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 *   See the License for the specific language governing permissions and        *
 *   limitations under the License.                                             *
 *                                                                              *
 *******************************************************************************/

#pragma once

#include <array>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <optional>
#include <vector>
#include <queue>
#include <functional>
#include <optional>
#include <thread>
#include <memory>
#include <type_traits>
/*
This is a thread-safe SPSC-style pipeline with overwrite semantics.

Suggested usage:

PipelineSlot<std::vector<int>> pipeline;
pipeline.emplace({1,2,3}); // pipeline now holds {1,2,3}
auto i = pipeline.pop();  // i is std::optional<{1,2,3}>

std::vector<int> p{3,2,1}
pipeline.push(p); // pipeline holds a copy of p
pipeline.push(std::move(p)) // pipeline holds p
auto v = pipeline.pop(); // pipeline is empty. v holds p

pipeline.notify_exit();
pipeline.pop(); // returns std::nullopt


*/
template <typename T>
class PipelineSlot
{
public:
    static_assert(std::is_default_constructible_v<T>);

    PipelineSlot() = default;
    ~PipelineSlot()
    {
        this->notify_exit();
    }

public:
    PipelineSlot(const PipelineSlot &) = delete;
    PipelineSlot &operator=(const PipelineSlot &other) = delete;
    PipelineSlot(PipelineSlot &&) = delete;
    PipelineSlot &operator=(const PipelineSlot &&other) = delete;

public:
    // This overload is for std::vector
    // We just SISFINE our way out of errors
    template <typename U = T>
    std::enable_if_t<
        std::is_constructible_v<
            U,
            std::initializer_list<typename U::value_type>>,
        void>
    emplace(std::initializer_list<typename U::value_type> ilist)
    {
        std::unique_lock lck{this->mtx};
        T value{ilist};
        this->value = std::move(value);
        has_value_ = true;
        resource_notifier.notify_one();
    }

    template <typename... Args>
    void emplace(Args &&...args)
    {
        std::unique_lock lck{this->mtx};
        T value{std::forward<Args>(args)...};
        this->value = std::move(value);
        has_value_ = true;
        resource_notifier.notify_one();
    }

    void push(T &&value)
    {
        std::unique_lock lck{this->mtx};
        this->value = std::move(value);
        has_value_ = true;
        resource_notifier.notify_one();
    }

    void push(const T &value)
    {
        std::unique_lock lck{this->mtx};
        this->value = value;
        has_value_ = true;
        resource_notifier.notify_one();
    }

    bool has_value()
    {
        std::unique_lock lck{this->mtx};
        return this->has_value_;
    }

    // Will only return std::nullopt
    // If notify_exit is called
    std::optional<T> pop()
    {
        std::unique_lock lck{this->mtx};
        resource_notifier.wait(lck, [&]
                               { return this->has_value_ || this->do_exit; });

        if (this->do_exit)
            return std::nullopt;

        this->has_value_ = false;
        return std::move(value);
    }
    void notify_exit()
    {
        this->do_exit = true;
        this->resource_notifier.notify_all();
    }

    operator bool() const
    {
        return !do_exit;
    }

private:
    T value;
    std::mutex mtx;
    std::condition_variable resource_notifier;
    bool has_value_{false};
    std::atomic<bool> do_exit{false};
};

template <typename T>
struct function_traits;

// For function pointers
template <typename R, typename Arg>
struct function_traits<R (*)(Arg)>
{
    using input_type = Arg;
    using output_type = R;
};

// For std::function
template <typename R, typename Arg>
struct function_traits<std::function<R(Arg)>>
{
    using input_type = Arg;
    using output_type = R;
};

// For lambdas and functors
template <typename T>
struct function_traits
{
private:
    using call_type = function_traits<decltype(&T::operator())>;

public:
    using input_type = typename call_type::input_type;
    using output_type = typename call_type::output_type;
};

// For lambda operator()
template <typename C, typename R, typename Arg>
struct function_traits<R (C::*)(Arg) const>
{
    using input_type = Arg;
    using output_type = R;
};

template <typename Fn>
class PipelineStep
{
public:
    using I = typename function_traits<Fn>::input_type;
    using O = typename function_traits<Fn>::output_type;

    using InputQueue = PipelineSlot<I>;
    using OutputQueue = PipelineSlot<O>;
    using OutputQueuePtr = std::conditional_t<
        std::is_void_v<O>,
        std::nullptr_t,
        std::shared_ptr<OutputQueue>>;

    // Constructor for non-void output types
    template <typename T = O, std::enable_if_t<!std::is_void_v<T>, int> = 0>
    PipelineStep(std::shared_ptr<InputQueue> in,
                 std::shared_ptr<OutputQueue> out,
                 Fn action)
        : input(std::move(in)), output(std::move(out)), fn(std::move(action)),
          worker(&PipelineStep::run, this)
    {
        if (!output)
            throw std::invalid_argument("Output queue must not be null for non-void functions.");
    }

    // Constructor for void output type
    template <typename T = O, std::enable_if_t<std::is_void_v<T>, int> = 0>
    PipelineStep(std::shared_ptr<InputQueue> in,
                 Fn action)
        : input(std::move(in)), output(nullptr), fn(std::move(action)),
          worker(&PipelineStep::run, this)
    {
    }

    ~PipelineStep() { stop(); }

    void stop()
    {
        input->notify_exit();
        if constexpr (!std::is_void_v<O>)
            output->notify_exit();
        worker.join();
    }

private:
    std::shared_ptr<InputQueue> input;
    OutputQueuePtr output;
    Fn fn;
    std::thread worker;

    void run()
    {

        if constexpr (!std::is_void_v<O>)
        {
            while (*input && *output)
            {
                auto v{input->pop()};
                if (v.has_value())
                {
                    if constexpr (std::is_move_constructible_v<I> && std::is_move_assignable_v<O>)
                    {
                        auto rv = fn(std::move(v.value()));
                        output->push(std::move(rv));
                    }
                    else
                    {
                        auto rv = fn(v.value());
                        output->push(rv);
                    }
                }
            }
            input->notify_exit();
            output->notify_exit();
        }
        else
        {
            while (*input)
            {
                auto v = input->pop();
                if (v.has_value())
                {

                    fn(std::move(v.value()));
                }
            }
            input->notify_exit();
        }
    }
};

template <typename... T>
class Pipeline
{
public:
    using TupleType = std::tuple<T...>;
    explicit Pipeline(TupleType &&tup) : data(std::move(tup)) {}

    // template <typename C>
    // void push(C t)
    // {
    //     std::get<0>(data)->push(t);
    // }

    template <typename C>
    void push(C &&t)
    {
        std::get<0>(data)->push(t);
    }

    template <typename C>
    void push(const C &t)
    {
        std::get<0>(data)->push(t);
    }

private:
    TupleType data;
};

// Base case: last stage returns void, so only input queue and Fn
template <typename Fn>
auto make_pipeline_chain(std::shared_ptr<PipelineSlot<typename function_traits<Fn>::input_type>> input, Fn fn)
{
    using Step = PipelineStep<Fn>;
    return std::make_tuple(std::make_shared<Step>(input, std::move(fn)));
}

// Recursive case: function returns non-void, so we make next slot
template <typename Fn, typename... Rest>
auto make_pipeline_chain(std::shared_ptr<PipelineSlot<typename function_traits<Fn>::input_type>> input, Fn fn, Rest &&...rest)
{
    using O = typename function_traits<Fn>::output_type;
    using Step = PipelineStep<Fn>;

    auto output = std::make_shared<PipelineSlot<O>>();
    auto step = std::make_shared<Step>(input, output, std::move(fn));

    auto rest_chain = make_pipeline_chain(output, std::forward<Rest>(rest)...);

    return std::tuple_cat(std::make_tuple(step), rest_chain);
}

// Main entry point: user provides stages
template <typename FirstFn, typename... RestFns>
auto make_pipeline_impl(FirstFn first_fn, RestFns... rest_fns)
{
    using I = typename function_traits<FirstFn>::input_type;
    auto input = std::make_shared<PipelineSlot<I>>();

    auto chain = make_pipeline_chain(input, std::move(first_fn), std::move(rest_fns)...);

    return std::tuple_cat(std::make_tuple(input), chain);
}

// Main entry point: user provides stages
template <typename FirstFn, typename... RestFns>
auto make_pipeline(FirstFn first_fn, RestFns... rest_fns)
{
    return Pipeline{std::move(make_pipeline_impl(std::move(first_fn), std::move(rest_fns)...))};
}
