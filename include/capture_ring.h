#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Single-producer / single-consumer byte ring holding whole serialized records.
//
// The producer appends a record atomically (all of it or none of it) and the
// consumer drains bytes in arbitrary-sized pieces, so it can feed a USB FIFO
// that accepts partial writes. Producer and consumer may run on different
// cores: each side writes only its own index and publishes it with
// release/acquire ordering.
//
// Capacity must be a power of two. Indices are free-running uint32_t counters,
// so used = head - tail stays correct across wraparound.
class CaptureRing {
 public:
  void init(uint8_t* storage, uint32_t capacity) {
    storage_ = storage;
    capacity_ = capacity;
    mask_ = capacity - 1;
    head_ = 0;
    tail_ = 0;
  }

  static constexpr bool capacityValid(uint32_t capacity) {
    return capacity != 0 && (capacity & (capacity - 1)) == 0;
  }

  uint32_t capacity() const { return capacity_; }

  uint32_t used() const {
    return loadAcquire(&head_) - loadAcquire(&tail_);
  }

  uint32_t freeSpace() const { return capacity_ - used(); }

  // Producer: append the concatenation of two segments as one record. Returns
  // false without writing anything when the ring lacks room for both.
  bool tryWrite(const void* a, uint32_t a_len, const void* b = nullptr, uint32_t b_len = 0) {
    const uint32_t head = head_;
    const uint32_t tail = loadAcquire(&tail_);
    const uint32_t total = a_len + b_len;
    if (total > capacity_ - (head - tail)) {
      return false;
    }
    copyIn(head, a, a_len);
    if (b_len > 0) {
      copyIn(head + a_len, b, b_len);
    }
    storeRelease(&head_, head + total);
    return true;
  }

  // Consumer: copy up to len bytes starting `offset` bytes past the read
  // position without consuming them. Returns the number of bytes copied.
  uint32_t peek(void* out, uint32_t len, uint32_t offset = 0) const {
    const uint32_t avail = used();
    if (offset >= avail) {
      return 0;
    }
    if (len > avail - offset) {
      len = avail - offset;
    }
    copyOut(tail_ + offset, out, len);
    return len;
  }

  // Consumer: pointer to the longest contiguous readable span (stops at the
  // wrap point). Returns its length; 0 when empty.
  uint32_t readSpan(const uint8_t** out) const {
    const uint32_t avail = used();
    const uint32_t start = tail_ & mask_;
    const uint32_t to_end = capacity_ - start;
    *out = storage_ + start;
    return avail < to_end ? avail : to_end;
  }

  // Consumer: release len bytes that were peeked or read via readSpan().
  void consume(uint32_t len) {
    storeRelease(&tail_, tail_ + len);
  }

  // Consumer: drop everything currently queued. Safe while the producer runs
  // because it only moves the consumer's own index.
  void discardAll() {
    storeRelease(&tail_, loadAcquire(&head_));
  }

 private:
  static uint32_t loadAcquire(const volatile uint32_t* p) {
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
  }

  static void storeRelease(volatile uint32_t* p, uint32_t v) {
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
  }

  void copyIn(uint32_t pos, const void* src, uint32_t len) {
    const uint32_t start = pos & mask_;
    const uint32_t first = (len < capacity_ - start) ? len : capacity_ - start;
    memcpy(storage_ + start, src, first);
    if (len > first) {
      memcpy(storage_, static_cast<const uint8_t*>(src) + first, len - first);
    }
  }

  void copyOut(uint32_t pos, void* dst, uint32_t len) const {
    const uint32_t start = pos & mask_;
    const uint32_t first = (len < capacity_ - start) ? len : capacity_ - start;
    memcpy(dst, storage_ + start, first);
    if (len > first) {
      memcpy(static_cast<uint8_t*>(dst) + first, storage_, len - first);
    }
  }

  uint8_t* storage_ = nullptr;
  uint32_t capacity_ = 0;
  uint32_t mask_ = 0;
  volatile uint32_t head_ = 0;
  volatile uint32_t tail_ = 0;
};
