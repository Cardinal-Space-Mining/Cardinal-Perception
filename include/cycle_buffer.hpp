#include <mutex>


template<typename T>
class StackCycleBuffer
{
public:
    inline StackCycleBuffer() :
        data{ T{}, T{} },
        ptr{ data + 0, data + 1 },
        mtx{ std::mutex{}, std::mutex{} }
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

    void swap()
    {
        bool locked_0 = this->mtx[0].try_lock();
        this->mtx[1].lock();
        if(!locked_0) this->mtx[0].lock();

        std::swap(this->ptr[0], this->ptr[1]);

        this->mtx[0].unlock();
        this->mtx[1].unlock();
    }
    bool try_swap()
    {
        bool locked_0 = this->mtx[0].try_lock();
        bool locked_1 = this->mtx[1].try_lock();
        if(locked_0 && locked_1)
        {
            std::swap(this->ptr[0], this->ptr[1]);
            this->mtx[0].unlock();
            this->mtx[1].unlock();
            return true;
        }
        if(locked_0) this->mtx[0].unlock();
        if(locked_1) this->mtx[1].unlock();
        return false;
    }

protected:
    T& _access(std::unique_lock& l, size_t idx)
    {
        if(&l.mutex() == &this->mtx[idx])
        {
            if(!l.is_locked()) l.lock();
        }
        else
        {
            l = std::unique_lock{ this->mtx[idx] };
        }
        return *this->ptr[idx];
    }
    T* _try_access(std::unique_lock& l, size_t idx)
    {
        if(&l.mutex() == &this->mtx[idx])
        {
            if(!l.is_locked()) l.try_lock();
        }
        else
        {
            l = std::unique_lock{ this->mtx[idx], std::try_to_lock };
        }

        if(l.is_locked()) return &this->ptr[idx];
        else return nullptr;
    }

private:
    T data[2];
    T* ptr[2];
    std::mutex mtx[2];

};
