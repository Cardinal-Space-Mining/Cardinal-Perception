#pragma once

#include <vector>
#include <limits>
#include <type_traits>
#include <cassert>

template<
    class Key,                          // priority key type (float, int, etc.)
    class Index = int,                  // ID type (int32_t, etc.)
    class Comparator = std::less<Key>,  // Min-heap by default
    int D = 4>                          // arity: default 4
class DaryHeap
{
    static_assert(D >= 2, "D-ary heap requires D >= 2");
    static_assert(std::is_integral<Index>::value, "Index must be integral");

public:
    using key_type = Key;
    using index_type = Index;
    using size_type = std::size_t;

    explicit DaryHeap(Index max_id = 0, const Comparator& comp = Comparator()) :
        _comp(comp)
    {
        if (max_id > 0)
        {
            reserve_ids(max_id + 1);
        }
    }

    // Pre-allocate space for IDs in [0, max_id]
    void reserve_ids(Index max_id_plus_one)
    {
        const size_type n = static_cast<size_type>(max_id_plus_one);
        _pos.assign(n, kAbsent);
        _keys.assign(n, Key{});
    }

    // Reserve heap capacity
    void reserve_heap(size_type cap) { _heap.reserve(cap); }

    bool empty() const noexcept { return _heap.empty(); }
    size_type size() const noexcept { return _heap.size(); }

    // Is id currently in the heap?
    bool contains(Index id) const noexcept
    {
        return id_in_range(id) && _pos[static_cast<size_type>(id)] != kAbsent;
    }

    // Return the id with best key (minimum for Comparator=less)
    Index top() const
    {
        assert(!empty());
        return _heap[0];
    }

    const Key& top_key() const
    {
        assert(!empty());
        return _keys[_heap[0]];
    }

    // Insert a new (id, key)
    void push(Index id, const Key& key)
    {
        ensure_id(id);
        assert(!contains(id));
        _keys[static_cast<size_type>(id)] = key;
        _heap.push_back(id);
        _pos[static_cast<size_type>(id)] = static_cast<Index>(_heap.size() - 1);
        sift_up(_heap.size() - 1);
    }

    // Extract-best (min for Comparator=less)
    Index pop()
    {
        assert(!empty());
        Index root = _heap[0];
        remove_at_root();
        return root;
    }

    // Remove id if present; returns true if removed.
    bool erase(Index id)
    {
        if (!contains(id))
        {
            return false;
        }
        size_type i = static_cast<size_type>(_pos[id]);
        remove_at(i);
        return true;
    }

    // Update the key and restore heap order in the right direction.
    // If you know it only decreased/increased, call the specialized version.
    void update_key(Index id, const Key& new_key)
    {
        assert(contains(id));
        size_type i = static_cast<size_type>(_pos[id]);
        const Key& old = _keys[id];
        _keys[id] = new_key;
        if (is_better(new_key, old))
        {
            sift_up(i);
        }
        else if (is_better(old, new_key))
        {
            sift_down(i);
        }
        // equal: no-op
    }

    // Strict decrease (if new_key is known to be better)
    void decrease_key(Index id, const Key& new_key)
    {
        assert(contains(id));
        size_type i = static_cast<size_type>(_pos[id]);
        // For min-heap, new_key must be strictly better than old
        assert(is_better(new_key, _keys[id]));
        _keys[id] = new_key;
        sift_up(i);
    }

    // Strict increase (if new_key is known to be worse)
    void increase_key(Index id, const Key& new_key)
    {
        assert(contains(id));
        size_type i = static_cast<size_type>(_pos[id]);
        assert(is_better(_keys[id], new_key));
        _keys[id] = new_key;
        sift_down(i);
    }

    // Access or set key of an id (valid even if not contained).
    const Key& key(Index id) const
    {
        ensure_id(id);
        return _keys[static_cast<size_type>(id)];
    }
    void set_key(Index id, const Key& k)
    {
        ensure_id(id);
        _keys[static_cast<size_type>(id)] = k;
    }

private:
    // Storage:
    //   heap:      array of ids in heap order
    //   pos[id]:   index in heap[] or kAbsent if not present
    //   keys[id]:  current key associated with id
    std::vector<Index> _heap;
    std::vector<Index> _pos;
    std::vector<Key> _keys;
    Comparator _comp;

    static constexpr Index kAbsent = static_cast<Index>(-1);

    // Helpers
    static constexpr size_type parent(size_type i) noexcept
    {
        return (i - 1) / D;
    }
    static constexpr size_type child0(size_type i) noexcept
    {
        return D * i + 1;
    }

    bool id_in_range(Index id) const noexcept
    {
        return static_cast<size_type>(id) < _pos.size();
    }

    void ensure_id(Index id)
    {
        if (!id_in_range(id))
        {
            // Grow pos/keys to accommodate id
            size_type new_size = static_cast<size_type>(id) + 1;
            _pos.resize(new_size, kAbsent);
            _keys.resize(new_size);
        }
    }

    inline bool better(Index a, Index b) const noexcept
    {
        // a has higher priority than b if comp(key[a], key[b]) is true
        return _comp(_keys[a], _keys[b]);
    }
    inline bool is_better(const Key& a, const Key& b) const noexcept
    {
        return _comp(a, b);
    }

    void swap_nodes(size_type i, size_type j) noexcept
    {
        Index ai = _heap[i], aj = _heap[j];
        std::swap(_heap[i], _heap[j]);
        _pos[ai] = static_cast<Index>(j);
        _pos[aj] = static_cast<Index>(i);
    }

    void sift_up(size_type i) noexcept
    {
        while (i > 0)
        {
            size_type p = parent(i);
            if (!better(_heap[i], _heap[p]))
            {
                break;
            }
            swap_nodes(i, p);
            i = p;
        }
    }

    void sift_down(size_type i) noexcept
    {
        const size_type n = _heap.size();
        for (;;)
        {
            size_type best = i;
            size_type c = child0(i);
            // Check up to D children
            for (int k = 0; k < D; ++k, ++c)
            {
                if (c < n && better(_heap[c], _heap[best]))
                {
                    best = c;
                }
            }
            if (best == i)
            {
                break;
            }
            swap_nodes(i, best);
            i = best;
        }
    }

    void remove_at_root() noexcept
    {
        const size_type last = _heap.size() - 1;
        Index root_id = _heap[0];
        _pos[root_id] = kAbsent;
        if (last == 0)
        {
            _heap.pop_back();
            return;
        }
        _heap[0] = _heap[last];
        _pos[_heap[0]] = 0;
        _heap.pop_back();
        sift_down(0);
    }

    void remove_at(size_type i) noexcept
    {
        const size_type last = _heap.size() - 1;
        Index id = _heap[i];
        _pos[id] = kAbsent;
        if (i == last)
        {
            _heap.pop_back();
            return;
        }
        _heap[i] = _heap[last];
        _pos[_heap[i]] = static_cast<Index>(i);
        _heap.pop_back();
        // Reorder both directions conservatively
        if (i > 0 && better(_heap[i], _heap[parent(i)]))
        {
            sift_up(i);
        }
        else
        {
            sift_down(i);
        }
    }
};
