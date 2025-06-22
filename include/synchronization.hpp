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

/*
This is a zero copy synchronization "MPMC", that drops old data for new data when available.

Suggested usage:

ResourcePipeline<std::vector<int>> pipeline;
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
class ResourcePipeline
{
public:
    ResourcePipeline() = default;
    ~ResourcePipeline() = default;

public:
    ResourcePipeline(const ResourcePipeline &) = delete;
    ResourcePipeline &operator=(const ResourcePipeline &other) = delete;
    ResourcePipeline(ResourcePipeline &&) = delete;
    ResourcePipeline &operator=(const ResourcePipeline &&other) = delete;

public:
    // This overload is for std::vector
    // We just SISFINE our way out of errors
    template <typename U = T>
    std::enable_if_t<std::is_constructible_v<U, std::initializer_list<typename U::value_type>>, void>
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
        if (this->do_exit)
        {
            return std::nullopt;
        }
        if (!this->has_value_)
        {
            resource_notifier.wait(lck);
        }
        if (this->do_exit)
        {
            return std::nullopt;
        }
        this->has_value_ = false;
        return std::move(value);
    }

    void notify_exit()
    {
        this->do_exit = true;
        this->resource_notifier.notify_all();
    }

private:
    T value;
    std::mutex mtx;
    std::condition_variable resource_notifier;
    bool has_value_{false};
    std::atomic<bool> do_exit{false};
};
