#pragma once
#include <cstddef>
#include <cassert>

// Fixed-capacity sliding window, backed by a plain array.
// Pushing into a full window overwrites the oldest value (ring buffer).
// Index 0 is the oldest element, size()-1 the newest.
template <std::size_t N>
class MySlidingWindow {
    static_assert(N > 0, "capacity must be > 0");

public:
    // Add a value. Returns true if an old value was evicted (written to *evicted if given).
    bool push(uint32_t x, uint32_t* evicted = nullptr) {
        if (count_ < N) {
            data_[(head_ + count_) % N] = x;
            ++count_;
            return false;
        }
        if (evicted) *evicted = data_[head_];
        data_[head_] = x;
        head_ = (head_ + 1) % N;
        return true;
    }

    uint32_t operator[](std::size_t i) const {
        assert(i < count_);
        return data_[(head_ + i) % N];
    }

    std::size_t size() const { return count_; }
    static constexpr std::size_t capacity() { return N; }
    bool empty() const { return count_ == 0; }
    bool full()  const { return count_ == N; }
    void clear() { head_ = 0; count_ = 0; }

    uint32_t sum() const {
        uint32_t s = 0;
        for (std::size_t i = 0; i < count_; ++i) s += (*this)[i];
        return s;
    }
    uint32_t mean() const { return count_ ? sum() / count_ : 0; }

private:
    uint32_t data_[N] = {};
    std::size_t head_    = 0;  // index of oldest element
    std::size_t count_   = 0;
};
