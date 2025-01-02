/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*                                ;xxxxxxx:                                     *
*                               ;$$$$$$$$$       ...::..                       *
*                               $$$$$$$$$$x   .:::::::::::..                   *
*                            x$$$$$$$$$$$$$$::::::::::::::::.                  *
*                        :$$$$$&X;      .xX:::::::::::::.::...                 *
*                .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :                *
*               :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.               *
*              :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.               *
*             ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::                *
*              X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.                *
*               .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                 *
*                X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                   *
*                $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                     *
*                $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                     *
*                $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                     *
*                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
*                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
*              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
*             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
*              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
*               :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                     *
*               ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                      *
*              ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                             *
*              ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                                *
*              :;;;;;;;;;;;;.  :$$$$$$$$$$X                                    *
*               .;;;;;;;;:;;    +$$$$$$$$$                                     *
*                 .;;;;;;.       X$$$$$$$:                                     *
*                                                                              *
*******************************************************************************/

#pragma once

#include <array>
#include <mutex>
#include <atomic>
#include <condition_variable>


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
    /* Aquire a reference to the current input buffer. Internally locks a control mutex,
     * so unlockInput() or unlockInputAndNotify() must be called when buffer modification
     * is complete on the current thread to unlock it! */
    T& aquireInput()
    {
        this->swap_mtx.lock();
        return this->input();
    }
    /* Return the modified input and unlock the internal mutex WITHOUT notifying waiting threads.
     * Useful when no modifications occured to the buffer. */
    void unlockInput(const T& v)
    {
        if(&v == &this->input())
        {
            this->swap_mtx.unlock();
        }
    }
    /* Return the modified input while unlocking the internal mutex and notifying waiting threads. */
    void unlockInputAndNotify(const T& v)
    {
        if(&v == &this->input())
        {
            this->swap_mtx.unlock();
            this->resource_available = true;
            this->resource_notifier.notify_all();
        }
    }
    /* Copies the input buffer and notifies waiting threads. Internal mutex MUST BE UNLOCKED before
     * calling this method as both locking and unlocking is handled internally here. */
    void updateAndNotify(const T& v)
    {
        this->swap_mtx.lock();
        this->input() = v;
        this->swap_mtx.unlock();
        this->resource_available = true;
        this->resource_notifier.notify_all();
    }

    /* Access the output buffer. */
    const T& aquireOutput() const
    {
        return this->output();
    }
    /* Aquire the newest output if available and access the output buffer. */
    const T& aquireNewestOutput() const
    {
        if(this->resource_available)
        {
            this->swap_mtx.lock();
            this->input_index ^= 0x1;
            this->swap_mtx.unlock();
            this->resource_available = false;
        }
        return this->output();
    }
    /* Has the resource been updated on another thread? */
    bool hasUpdatedResource() const
    {
        return this->resource_available;
    }
    /* Wait for a new resource to become available and return output buffer when this occurs.
     * Note that this method may also return if notifyExit() has been called in which case it
     * is advised to check for an exit state (external) - the output buffer will not be new in
     * this case. */
    const T& waitNewestResource() const
    {
        std::unique_lock<std::mutex> l{ this->swap_mtx };
        while(!this->do_exit && !this->resource_available)
        {
            this->resource_notifier.wait(l);
        }
        this->do_exit = false;
        return this->aquireNewestOutput();
    }

    /* Unblock waiting threads in the case of an exit. */
    void notifyExit() const
    {
        this->do_exit = true;
        this->resource_notifier.notify_all();
    }

protected:
    inline T& input()
        { return this->buffer[static_cast<size_t>(input_index)]; }
    inline T& output()
        { return this->buffer[static_cast<size_t>(input_index ^ 0x1)]; }

protected:
    std::array<T, 2> buffer;
    mutable uint32_t input_index{ 0 };
    mutable std::mutex swap_mtx;
    mutable std::condition_variable resource_notifier;
    mutable std::atomic<bool>
        resource_available{ false },
        do_exit{ false };

};


// template<typename T>
// struct Synchronized
// {
// public:
//     inline Synchronized() noexcept : item{}, mtx{} {}
//     inline Synchronized(const Synchronized& ref) noexcept : item{ ref.item }, mtx{} {}
//     inline Synchronized(Synchronized&& ref) noexcept : item{ std::move(ref.item) }, mtx{ std::move(ref.mtx) } {}
//     ~Synchronized() = default;

//     inline T& lock(std::unique_lock<std::mutex>& l)
//     {
//         if(l.mutex() == &this->mtx)
//         {
//             if(!l.owns_lock()) l.lock();
//         }
//         else
//         {
//             l = std::unique_lock<std::mutex>{ this->mtx };
//         }
//         return this->item;
//     }
//     inline T* try_lock(std::unique_lock<std::mutex>& l)
//     {
//         if(l.mutex() == &this->mtx)
//         {
//             if(!l.owns_lock()) l.try_lock();
//         }
//         else
//         {
//             l = std::unique_lock<std::mutex>{ this->mtx, std::try_to_lock };
//         }
//         if(l.owns_lock()) return &this->item;
//         else return nullptr;
//     }

// private:
//     T item;
//     std::mutex mtx;

// };

// template<typename T>
// class SpinBuffer
// {
// public:
//     inline SpinBuffer() :
//         data{ T{}, T{} },
//         ptr{ data + 0, data + 1 },
//         swap_hint{ false }
//         {}
//     inline SpinBuffer(const SpinBuffer<T>& ref) :
//         data{ ref.data[0], ref.data[1] },
//         ptr{ data + 0, data + 1 },
//         // mtx{ std::mutex{}, std::mutex{} },
//         swap_hint{ ref.swap_hint.load() }
//         {}

//     inline T& A(std::unique_lock<std::mutex>& l)
//     {
//         return this->_access(l, 0);
//     }
//     inline T& B(std::unique_lock<std::mutex>& l)
//     {
//         return this->_access(l, 1);
//     }

//     inline T* try_A(std::unique_lock<std::mutex>& l)
//     {
//         return this->_try_access(l, 0);
//     }
//     inline T* try_B(std::unique_lock<std::mutex>& l)
//     {
//         return this->_try_access(l, 1);
//     }

//     void spin()
//     {
//         bool locked_0 = this->mtx[0].try_lock();
//         this->mtx[1].lock();
//         if(!locked_0) this->mtx[0].lock();

//         std::swap(this->ptr[0], this->ptr[1]);
//         this->swap_hint = false;

//         this->mtx[0].unlock();
//         this->mtx[1].unlock();
//     }
//     bool try_spin()
//     {
//         bool locked_0 = this->mtx[0].try_lock();
//         bool locked_1 = this->mtx[1].try_lock();
//         if(locked_0 && locked_1)
//         {
//             std::swap(this->ptr[0], this->ptr[1]);
//             this->swap_hint = false;

//             this->mtx[0].unlock();
//             this->mtx[1].unlock();

//             return true;
//         }
//         this->swap_hint = true;
//         if(locked_0) this->mtx[0].unlock();
//         if(locked_1) this->mtx[1].unlock();
//         return false;
//     }

// protected:
//     T& _access(std::unique_lock<std::mutex>& l, size_t idx)
//     {
//         if(this->swap_hint)
//         {
//             this->try_spin();
//         }
//         if(l.mutex() == &this->mtx[idx])
//         {
//             if(!l.owns_lock()) l.lock();
//         }
//         else
//         {
//             l = std::unique_lock<std::mutex>{ this->mtx[idx] };
//         }
//         return *this->ptr[idx];
//     }
//     T* _try_access(std::unique_lock<std::mutex>& l, size_t idx)
//     {
//         if(this->swap_hint)
//         {
//             this->try_spin();
//         }
//         if(l.mutex() == &this->mtx[idx])
//         {
//             if(!l.owns_lock()) l.try_lock();
//         }
//         else
//         {
//             l = std::unique_lock<std::mutex>{ this->mtx[idx], std::try_to_lock };
//         }

//         if(l.owns_lock()) return &this->ptr[idx];
//         else return nullptr;
//     }

// private:
//     T data[2];
//     T* ptr[2];  // TODO: we can do this with a single idx
//     std::array<std::mutex, 2> mtx;
//     std::atomic<bool> swap_hint;

// };
