#pragma once

#include <iostream>
#include <shared_mutex>
#include <mutex>
#include <memory>
#include <toml++/toml.h>

#include "logger/logger.hpp"
#include "utility.hpp"

namespace luhsoccer::config_provider {

/**
 * @brief A Class used to provide a getter and a setter for a value (multithreading safe)
 *
 * @tparam T The type of the stored Value
 * @param writable true if the value should be editbale during runtime, false otherwise
 */
template <typename T>
class ValueStorage {
   public:
    ValueStorage(T val, bool writable) : value(val), default_value(std::move(val)), writable(writable) {}

    // TODO: replace with operator overload ?
    const T& val() const {
        std::shared_lock<std::shared_mutex> shared_lock(s_mtx);
        return this->value;
    }

    // TODO: check for min/max values ?
    virtual bool set(const T& new_val) {
        if (!this->writable) {
            LOG_WARNING(logger::Logger("config_provider"), "Cant Set Value to '{}' because variable is not writable",
                        new_val);
            return false;
        }

        this->forceSet(new_val);
        return true;
    }

    [[nodiscard]] bool isWritable() const { return this->writable; }
    [[nodiscard]] const T& getDefaultValue() const { return this->default_value; }
    [[nodiscard]] bool hasValueChanged() const { return this->value_changed; }

    operator T() { return this->val(); }

   protected:
    /**
     * @brief Resets the value of this parameter back to the default value
     * Additionally removes the entry from the diff-table
     *
     */
    void resetValue() { this->set(this->default_value); }

    void forceSet(const T& new_val) {
        std::unique_lock<std::shared_mutex> exclusive_lock(s_mtx);
        this->value = new_val;
        this->checkValueChangedFlag();
    }

    void setDefaultValue(const T& new_val) {
        this->default_value = new_val;
        this->checkValueChangedFlag();
    }

    void checkValueChangedFlag() { this->value_changed = util::notEqual<T>(this->value, this->default_value); }

   protected:
    T value;
    T default_value;
    bool value_changed = false;

   private:
    bool writable;
    mutable std::shared_mutex s_mtx;
};

/**
 * @brief A Class that holds getters for two constant values
 *
 * @tparam T The Type of the values
 */
template <typename T>
class MinMaxValStorage : public ValueStorage<T> {
   public:
    MinMaxValStorage(T val, T min, T max, bool writable) : ValueStorage<T>(val, writable), min(min), max(max) {}

    //[[nodiscard]] bool isWritable() { return ValueStorage<T>::isWritable(); }

    bool set(const T& new_val) override {
        if (new_val < this->getMin() || new_val > this->getMax()) {
            LOG_WARNING(logger::Logger("config_provider"),
                        "The Given value '{}' was not in the min/max range of [ {}, {} ]", new_val, this->getMin(),
                        this->getMax());
            return false;
        }

        ValueStorage<T>::set(new_val);
        return true;
    }

    [[nodiscard]] T getMin() const { return this->min; }
    [[nodiscard]] T getMax() const { return this->max; }

   private:
    const T min;
    const T max;
};
}  // namespace luhsoccer::config_provider
