/**
 * @file circular_buffer.h
 * @author Fabrice Zeug (zeug@stud.uni-hannover.de)
 * @brief CircularBuffer that is thread save. Elements in Buffer can't be edited
 * only new data can be pushed.
 * @version 0.1
 * @date 2022-08-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace luhsoccer {

/**
 * @brief minimum buffer size that is always enforced
 *
 */
constexpr size_t MIN_BUFFER_SIZE = 2;

/**
 * @brief CircularBuffer that is thread save. Elements in Buffer can't be edited
 * only new data can be pushed.
 *
 * @tparam T Object of Buffer
 *
 */
template <class T>
class CircularBuffer {
   public:
    /**
     * @brief Construct a new Circular Buffer object
     *
     * @param size Size of Buffer
     * @param previous_buffer Pointer to a previous buffer
     */
    inline CircularBuffer(size_t size, std::shared_ptr<const CircularBuffer<T>> previous_buffer = nullptr)
        : buffer(std::max(size + 1, MIN_BUFFER_SIZE)),
          head(0),
          loop_counter(0),
          previous_buffer(previous_buffer),
          point_of_entry(previous_buffer ? previous_buffer->head.load() : 0),
          previous_loop_counter(previous_buffer ? previous_buffer->loop_counter.load() : 0) {}
    /**
     * @brief Construct a new Circular Buffer with a default value
     *
     * @param size Size of the Buffer
     * @param default_value default object that is copied in every slot
     */
    inline CircularBuffer(size_t size, const T& default_value) : CircularBuffer(size) {
        std::fill_n(this->buffer.begin(), this->buffer.size(), default_value);
        this->loop_counter++;
    }

    CircularBuffer(const CircularBuffer&) = delete;
    CircularBuffer& operator=(const CircularBuffer&) = delete;

    CircularBuffer(const CircularBuffer&&) = delete;
    CircularBuffer& operator=(CircularBuffer&&) = delete;
    ~CircularBuffer() = default;

    /**
     * @brief get the size of Buffer

     */
    [[nodiscard]] size_t size() const {
        return this->getSizeOfThisBuffer(this->head, this->loop_counter) + this->getSizeOfPreviousBuffer();
    }

    /**
     * @brief push new data to the Buffer
     *
     * @param element
     */
    inline void push(const T& element) {
        const std::lock_guard<std::mutex> lock(this->write_mutex);

        size_t new_head = (this->head + 1) % this->buffer.size();
        // NOLINTNEXTLINE - index always in bound due to mod
        this->buffer[new_head] = element;
        if (new_head == 0) {
            this->loop_counter++;
            this->previous_buffer = nullptr;
        }
        this->head = new_head;
    }

    /**
     * @brief get data from the buffer, where 0 is the latest data
     *
     * @param index index of data
     * @return T copy of object at index
     */
    [[nodiscard]] inline const T& at(size_t index, std::optional<size_t> entry_point = std::nullopt) const {
        size_t head = this->head;
        if (entry_point.has_value()) head = entry_point.value();
        if (index >= this->size()) {
            throw std::out_of_range("Size of Circular Buffer is only " + std::to_string(this->size()) +
                                    " elements, but index " + std::to_string(index) + " was requested");
        }
        size_t size_of_this_buffer = this->getSizeOfThisBuffer(head, this->loop_counter);
        if (previous_buffer && index >= size_of_this_buffer) {
            return this->previous_buffer->at(index - size_of_this_buffer, this->point_of_entry);
        } else {
            // NOLINTNEXTLINE - index always in bound due to mod
            return this->buffer[(head - index + this->buffer.size()) % this->buffer.size()];
        }
    }

   private:
    /**
     * @brief actual data
     *
     */
    std::vector<T> buffer;

    /**
     * @brief head pointer
     */
    std::atomic<size_t> head{};

    std::atomic_size_t loop_counter;

    /**
     * @brief mutex to write to the buffer
     */
    std::mutex write_mutex;

    [[nodiscard]] size_t size(size_t entry_point, size_t entry_loop) const {
        return this->getSizeOfThisBuffer(entry_point, entry_loop) + this->getSizeOfPreviousBuffer();
    };

    [[nodiscard]] size_t getSizeOfThisBuffer(size_t entry_point, size_t entry_loop) const {
        if (this->loop_counter == entry_loop) {
            if (this->loop_counter != 0) {
                return buffer.size() - (this->head - entry_point) - 1;
            } else {
                return entry_point;
            }
        } else if (this->loop_counter == entry_loop + 1) {
            if (this->head >= entry_point) {
                return 0;
            } else {
                return entry_point - this->head - 1;
            }
        } else {
            return 0;
        }
    }
    /// @brief get the valid size of the previous buffer
    [[nodiscard]] size_t getSizeOfPreviousBuffer() const {
        if (this->previous_buffer) {
            return this->previous_buffer->size(this->point_of_entry, this->previous_loop_counter);
        } else {
            return 0;
        }
    }
    /// @brief pointer to previous buffer
    std::shared_ptr<const CircularBuffer<T>> previous_buffer;
    /// @brief position of the head when deriving from buffer
    std::atomic_size_t point_of_entry;
    /// @brief loop count when derived from buffer
    std::atomic_size_t previous_loop_counter;
};
}  // namespace luhsoccer
