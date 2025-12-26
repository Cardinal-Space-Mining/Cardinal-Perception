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


namespace util
{

/**
 * A.K.A. an "SPSC"
 * 
 * >>> USAGE >>>
 * ResourcePipeline<T> pipe{};
 * 
 * (Input side):
 * auto& x = pipe.aquireInput();
 * < modify x... >
 * pipe.unlockInputAndNotify(x);
 * -- OR --
 * pipe.updateAndNotify(< new buffer >);
 * 
 * (Output side):
 * auto& x = pipe.waitNewestResource();
 * < check exit state >
 * 
 * (On exit):
 * pipe.notifyExit();
 */
template<typename T>
class ResourcePipeline
{
public:
    ResourcePipeline() = default;
    ResourcePipeline(const ResourcePipeline&) = delete;
    ResourcePipeline(ResourcePipeline&&) = delete;
    ~ResourcePipeline() = default;

public:
    /* Aquire a reference to the current input buffer. Internally locks a control
     * mutex, so unlockInput() or unlockInputAndNotify() must be called when
     * buffer modification is complete on the current thread to unlock it! */
    T& lockInput()
    {
        this->swap_mtx.lock();
        return this->input();
    }
    /* Unlock the internal mutex WITHOUT notifying waiting threads.
     * Useful when no modifications occured to the buffer. */
    void unlockInput(const T& v)
    {
        if (&v == &this->input())
        {
            this->swap_mtx.unlock();
        }
    }
    /* Unlock the internal mutex and notify waiting threads that the resources
     * has been updated. */
    void unlockInputAndNotify(const T& v)
    {
        if (&v == &this->input())
        {
            this->swap_mtx.unlock();
            this->resource_available = true;
            this->resource_notifier.notify_all();
        }
    }
    /* Copies the input buffer and notifies waiting threads. Internal mutex MUST
     * BE UNLOCKED before calling this method as both locking and unlocking is
     * handled internally here. */
    void updateAndNotify(const T& v)
    {
        this->swap_mtx.lock();
        this->input() = v;
        this->swap_mtx.unlock();
        this->resource_available = true;
        this->resource_notifier.notify_all();
    }

    /* Access the output buffer. */
    T& aquireOutput() { return this->output(); }
    /* Aquire the newest output if available and access the output buffer. */
    T& aquireNewestOutput()
    {
        if (this->resource_available)
        {
            this->swap_mtx.lock();
            this->input_index ^= 0x1;
            this->swap_mtx.unlock();
            this->resource_available = false;
        }
        return this->output();
    }
    /* Has the resource been updated on another thread? */
    bool hasUpdatedResource() const { return this->resource_available; }
    /* Wait for a new resource to become available and return the output buffer
     * when this occurs. Note that this method may also return if notifyExit() 
     * has been called in which case it is advised to check for an exit state
     * (external) - the output buffer will not be new in this case. */
    T& waitNewestResource()
    {
        std::mutex temp_mtx;
        std::unique_lock<std::mutex> l{temp_mtx};
        while (!this->do_exit && !this->resource_available)
        {
            this->resource_notifier.wait(l);
        }
        this->do_exit = false;
        return this->aquireNewestOutput();
    }

    /* Unblock waiting threads in the case of an exit. */
    void notifyExit()
    {
        this->do_exit = true;
        this->resource_notifier.notify_all();
    }

protected:
    inline T& input() { return this->buffer[static_cast<size_t>(input_index)]; }
    inline T& output()
    {
        return this->buffer[static_cast<size_t>(input_index ^ 0x1)];
    }

protected:
    std::array<T, 2> buffer;
    uint32_t input_index{0};
    std::mutex swap_mtx;
    std::condition_variable resource_notifier;
    std::atomic<bool> resource_available{false}, do_exit{false};
};

};
