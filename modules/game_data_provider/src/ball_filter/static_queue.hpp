#pragma once

#include <array>

namespace luhsoccer::game_data_provider::util {

template <size_t SIZE, typename T>
class StaticQueue {
   public:
    StaticQueue() = default;

    void push(const T& elem) {
        this->advanceIndex();
        this->buffer[this->current_index] = elem;
    }

    T& currentElement() { return this->buffer[this->current_index]; }

    [[nodiscard]] size_t currentIndex() const { return this->current_index; }

    const std::array<T, SIZE>& data() const { return this->buffer; }

    std::array<T, SIZE>& dataMut() { return this->buffer; }

    [[nodiscard]] constexpr size_t size() const { return SIZE; }

   private:
    void advanceIndex() { current_index = (current_index + 1) % SIZE; }

   private:
    size_t current_index = 0;
    std::array<T, SIZE> buffer{};
};

}  // namespace luhsoccer::game_data_provider::util