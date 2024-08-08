#pragma once

#include <mutex>
#include <atomic>
// #include <memory>


// template<typename T>
// struct ReferenceLock
// {
// friend class SpinBuffer<T>;
// protected:
//     inline ReferenceLock(const std::shared_ptr<T>& p, std::unique_lock&& l) : ptr{ p }, lock{ std::move(l) } {}
//     ~ReferenceLock() = default;     // unlocks as per unique_lock's destructor

// public:
//     inline bool isLocked() { return this->lock.owns_lock(); }
//     inline bool isValid() { return !this->ptr.expired(); }
//     inline bool operator bool() { return this->isLocked() && this->isValid(); }

//     inline void lock() { if(!this->isLocked()) this->lock.lock(); }
//     inline bool try_lock() { if(!this->isLocked()) return this->lock.try_lock(); }
//     inline void unlock() { if(this->isLocked()) this->lock.unlock(); }

//     inline T& ref()
//     {
//         if(*this) return *this->ptr.lock();
//         else throw std::exception();
//     }
//     inline T& operator*() { return this->ref(); }

// private:
//     std::weak_ptr<T> ptr;
//     std::unique_lock lock;

// };

template<typename T>
class SpinBuffer
{
public:
    inline SpinBuffer() :
        data{ { T{}, T{} } },
        ptr{ { data + 0, data + 1 } },
        mtx{ { std::mutex{}, std::mutex{} } },
        swap_hint{ false }
        {}

    inline T& A(std::unique_lock& l)
    {
        return this->_access(l, 0);
    }
    inline T& B(std::unique_lock& l)
    {
        return this->_access(l, 1);
    }

    inline T* try_A(std::unique_lock& l)
    {
        return this->_try_access(l, 0);
    }
    inline T* try_B(std::unique_lock& l)
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
    T& _access(std::unique_lock& l, size_t idx)
    {
        if(this->swap_hint)
        {
            this->try_spin();
        }
        if(&l.mutex() == &this->mtx[idx])
        {
            if(!l.owns_lock()) l.lock();
        }
        else
        {
            l = std::unique_lock{ this->mtx[idx] };
        }
        return *this->ptr[idx];
    }
    T* _try_access(std::unique_lock& l, size_t idx)
    {
        if(this->swap_hint)
        {
            this->try_spin();
        }
        if(&l.mutex() == &this->mtx[idx])
        {
            if(!l.owns_lock()) l.try_lock();
        }
        else
        {
            l = std::unique_lock{ this->mtx[idx], std::try_to_lock };
        }

        if(l.owns_lock()) return &this->ptr[idx];
        else return nullptr;
    }

private:
    T data[2];
    T* ptr[2];  // TODO: we can do this with a single idx
    std::mutex mtx[2];
    std::atomic<bool> swap_hint;

};
