/*******************************************************************************
*   Copyright (C) 2024 Cardinal Space Mining Club                              *
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
*                $$$::XXXXXXXXXXXXXXXXXXXXXX: :XXXXX; X$$;                     *
*                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
*                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
*              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
*             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
*              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
*               :$$$$$$$$$. +XXXXXXXXX:      ;: x$$$$$$$$$                     *
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


template<typename T>
struct Synchronized
{
public:
    inline Synchronized() noexcept : item{}, mtx{} {}
    inline Synchronized(const Synchronized& ref) noexcept : item{ ref.item }, mtx{} {}
    inline Synchronized(Synchronized&& ref) noexcept : item{ std::move(ref.item) }, mtx{ std::move(ref.mtx) } {}
    ~Synchronized() = default;

    inline T& lock(std::unique_lock<std::mutex>& l)
    {
        if(l.mutex() == &this->mtx)
        {
            if(!l.owns_lock()) l.lock();
        }
        else
        {
            l = std::unique_lock<std::mutex>{ this->mtx };
        }
        return this->item;
    }
    inline T* try_lock(std::unique_lock<std::mutex>& l)
    {
        if(l.mutex() == &this->mtx)
        {
            if(!l.owns_lock()) l.try_lock();
        }
        else
        {
            l = std::unique_lock<std::mutex>{ this->mtx, std::try_to_lock };
        }
        if(l.owns_lock()) return &this->item;
        else return nullptr;
    }

private:
    T item;
    std::mutex mtx;

};

template<typename T>
class SpinBuffer
{
public:
    inline SpinBuffer() :
        data{ T{}, T{} },
        ptr{ data + 0, data + 1 },
        swap_hint{ false }
        {}
    inline SpinBuffer(const SpinBuffer<T>& ref) :
        data{ ref.data[0], ref.data[1] },
        ptr{ data + 0, data + 1 },
        // mtx{ std::mutex{}, std::mutex{} },
        swap_hint{ ref.swap_hint.load() }
        {}

    inline T& A(std::unique_lock<std::mutex>& l)
    {
        return this->_access(l, 0);
    }
    inline T& B(std::unique_lock<std::mutex>& l)
    {
        return this->_access(l, 1);
    }

    inline T* try_A(std::unique_lock<std::mutex>& l)
    {
        return this->_try_access(l, 0);
    }
    inline T* try_B(std::unique_lock<std::mutex>& l)
    {
        return this->_try_access(l, 1);
    }

    void spin()
    {
        bool locked_0 = this->mtx[0].try_lock();
        this->mtx[1].lock();
        if(!locked_0) this->mtx[0].lock();

        std::swap(this->ptr[0], this->ptr[1]);
        this->swap_hint = false;

        this->mtx[0].unlock();
        this->mtx[1].unlock();
    }
    bool try_spin()
    {
        bool locked_0 = this->mtx[0].try_lock();
        bool locked_1 = this->mtx[1].try_lock();
        if(locked_0 && locked_1)
        {
            std::swap(this->ptr[0], this->ptr[1]);
            this->swap_hint = false;

            this->mtx[0].unlock();
            this->mtx[1].unlock();

            return true;
        }
        this->swap_hint = true;
        if(locked_0) this->mtx[0].unlock();
        if(locked_1) this->mtx[1].unlock();
        return false;
    }

protected:
    T& _access(std::unique_lock<std::mutex>& l, size_t idx)
    {
        if(this->swap_hint)
        {
            this->try_spin();
        }
        if(l.mutex() == &this->mtx[idx])
        {
            if(!l.owns_lock()) l.lock();
        }
        else
        {
            l = std::unique_lock<std::mutex>{ this->mtx[idx] };
        }
        return *this->ptr[idx];
    }
    T* _try_access(std::unique_lock<std::mutex>& l, size_t idx)
    {
        if(this->swap_hint)
        {
            this->try_spin();
        }
        if(l.mutex() == &this->mtx[idx])
        {
            if(!l.owns_lock()) l.try_lock();
        }
        else
        {
            l = std::unique_lock<std::mutex>{ this->mtx[idx], std::try_to_lock };
        }

        if(l.owns_lock()) return &this->ptr[idx];
        else return nullptr;
    }

private:
    T data[2];
    T* ptr[2];  // TODO: we can do this with a single idx
    std::array<std::mutex, 2> mtx;
    std::atomic<bool> swap_hint;

};
