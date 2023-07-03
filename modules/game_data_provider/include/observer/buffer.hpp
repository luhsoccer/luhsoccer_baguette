#pragma once

#include <array>
#include <shared_mutex>

namespace luhsoccer::observer::buffer {

/**
 * @brief A Class used to store elements in a Front-Back-Buffer Style
 *
 * @tparam T The Type of the Data that should be stored
 */
template <typename T>
class BackFrontBuffer {
   public:
    /**
     * @brief Construct a new Back Front Buffer with default initialized Objects
     *
     */
    BackFrontBuffer() { BackFrontBuffer(T{}, T{}); }

    /**
     * @brief Construct a new Back Front Buffer with two given Objects
     *
     * @param front_element The Object which is at the front in the beginning
     * @param back_element The Object which is at the back in the beginning
     */
    BackFrontBuffer(const T& front_element, const T& back_element) {
        data[0] = front_element;
        data[1] = back_element;
    }

    /**
     * @brief Gets the Element currently in the front (only a const reference to the front elment can be obtained)
     *
     * @return const T& A const reference to the Front element
     */
    [[nodiscard]] const T& getFrontBuffer() const {
        std::shared_lock shared_lock(buffer_mtx);
        return data[front];
    }

    /**
     * @brief Gets the Element currently in the back (a editable Reference can be obtained)
     *
     * @return T& A Reference to the element currently in the back
     */
    [[nodiscard]] T& getBackBuffer() {
        std::shared_lock shared_lock(buffer_mtx);
        return data[front ^ 1];
    }

    /**
     * @brief Used to switch the Front- and Back- Buffer
     *
     */
    void switchBuffers() {
        std::unique_lock unique_lock(buffer_mtx);
        front ^= 1;
    }

   private:
    /**
     * @brief A Mutex to lock the switching process. Because only the Front Buffer is obtainable by other threads and
     * only the back buffer
     *
     */
    mutable std::shared_mutex buffer_mtx;

    /**
     * @brief The index to the front element
     *
     */
    short front = 0;

    /**
     * @brief An array to store the front and back element
     *
     */
    std::array<T, 2> data;
};

}  // namespace luhsoccer::observer::buffer
