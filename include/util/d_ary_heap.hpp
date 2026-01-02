/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
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

#include <limits>
#include <vector>
#include <cassert>
#include <type_traits>


namespace util
{

template<
    class Key,                          // priority key type (float, int, etc.)
    class Index = int,                  // ID type (int32_t, etc.)
    class Comparator = std::less<Key>,  // Min-heap by default
    int D = 4>                          // arity: default 4
class DAryHeap
{
    static_assert(D >= 2, "D-ary heap requires D >= 2");
    static_assert(std::is_integral<Index>::value, "Index must be integral");

public:
    using key_type = Key;
    using index_type = Index;
    using size_type = std::size_t;

    explicit DAryHeap(Index max_id = 0, const Comparator& comp = Comparator()) :
        comp(comp)
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
        this->pos.assign(n, kAbsent);
        this->keys.assign(n, Key{});
    }

    // Reserve heap capacity
    void reserve_heap(size_type cap) { this->heap.reserve(cap); }

    bool empty() const noexcept { return this->heap.empty(); }
    size_type size() const noexcept { return this->heap.size(); }

    // Is id currently in the heap?
    bool contains(Index id) const noexcept
    {
        return id_in_range(id) &&
               this->pos[static_cast<size_type>(id)] != kAbsent;
    }

    // Return the id with best key (minimum for Comparator=less)
    Index top() const
    {
        assert(!empty());
        return this->heap[0];
    }

    const Key& top_key() const
    {
        assert(!empty());
        return this->keys[this->heap[0]];
    }

    // Insert a new (id, key)
    void push(Index id, const Key& key)
    {
        ensure_id(id);
        assert(!contains(id));
        this->keys[static_cast<size_type>(id)] = key;
        this->heap.push_back(id);
        this->pos[static_cast<size_type>(id)] =
            static_cast<Index>(this->heap.size() - 1);
        sift_up(this->heap.size() - 1);
    }

    // Extract-best (min for Comparator=less)
    Index pop()
    {
        assert(!empty());
        Index root = this->heap[0];
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
        size_type i = static_cast<size_type>(this->pos[id]);
        remove_at(i);
        return true;
    }

    // Update the key and restore heap order in the right direction.
    // If you know it only decreased/increased, call the specialized version.
    void update_key(Index id, const Key& new_key)
    {
        assert(contains(id));
        size_type i = static_cast<size_type>(this->pos[id]);
        const Key& old = this->keys[id];
        this->keys[id] = new_key;
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
        size_type i = static_cast<size_type>(this->pos[id]);
        // For min-heap, new_key must be strictly better than old
        assert(is_better(new_key, this->keys[id]));
        this->keys[id] = new_key;
        sift_up(i);
    }

    // Strict increase (if new_key is known to be worse)
    void increase_key(Index id, const Key& new_key)
    {
        assert(contains(id));
        size_type i = static_cast<size_type>(this->pos[id]);
        assert(is_better(this->keys[id], new_key));
        this->keys[id] = new_key;
        sift_down(i);
    }

    // Access or set key of an id (valid even if not contained).
    const Key& key(Index id) const
    {
        ensure_id(id);
        return this->keys[static_cast<size_type>(id)];
    }
    void set_key(Index id, const Key& k)
    {
        ensure_id(id);
        this->keys[static_cast<size_type>(id)] = k;
    }

private:
    // Storage:
    //   heap:      array of ids in heap order
    //   pos[id]:   index in heap[] or kAbsent if not present
    //   keys[id]:  current key associated with id
    std::vector<Index> heap;
    std::vector<Index> pos;
    std::vector<Key> keys;
    Comparator comp;

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
        return static_cast<size_type>(id) < this->pos.size();
    }

    void ensure_id(Index id)
    {
        if (!id_in_range(id))
        {
            // Grow pos/keys to accommodate id
            size_type new_size = static_cast<size_type>(id) + 1;
            this->pos.resize(new_size, kAbsent);
            this->keys.resize(new_size);
        }
    }

    inline bool better(Index a, Index b) const noexcept
    {
        // a has higher priority than b if comp(key[a], key[b]) is true
        return this->comp(this->keys[a], this->keys[b]);
    }
    inline bool is_better(const Key& a, const Key& b) const noexcept
    {
        return this->comp(a, b);
    }

    void swap_nodes(size_type i, size_type j) noexcept
    {
        Index ai = this->heap[i], aj = this->heap[j];
        std::swap(this->heap[i], this->heap[j]);
        this->pos[ai] = static_cast<Index>(j);
        this->pos[aj] = static_cast<Index>(i);
    }

    void sift_up(size_type i) noexcept
    {
        while (i > 0)
        {
            size_type p = parent(i);
            if (!better(this->heap[i], this->heap[p]))
            {
                break;
            }
            swap_nodes(i, p);
            i = p;
        }
    }

    void sift_down(size_type i) noexcept
    {
        const size_type n = this->heap.size();
        for (;;)
        {
            size_type best = i;
            size_type c = child0(i);
            // Check up to D children
            for (int k = 0; k < D; ++k, ++c)
            {
                if (c < n && better(this->heap[c], this->heap[best]))
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
        const size_type last = this->heap.size() - 1;
        Index root_id = this->heap[0];
        this->pos[root_id] = kAbsent;
        if (last == 0)
        {
            this->heap.pop_back();
            return;
        }
        this->heap[0] = this->heap[last];
        this->pos[this->heap[0]] = 0;
        this->heap.pop_back();
        sift_down(0);
    }

    void remove_at(size_type i) noexcept
    {
        const size_type last = this->heap.size() - 1;
        Index id = this->heap[i];
        this->pos[id] = kAbsent;
        if (i == last)
        {
            this->heap.pop_back();
            return;
        }
        this->heap[i] = this->heap[last];
        this->pos[this->heap[i]] = static_cast<Index>(i);
        this->heap.pop_back();
        // Reorder both directions conservatively
        if (i > 0 && better(this->heap[i], this->heap[parent(i)]))
        {
            sift_up(i);
        }
        else
        {
            sift_down(i);
        }
    }
};

};
