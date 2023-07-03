#pragma once

#include <functional>
#include <memory>
#include <list>

namespace luhsoccer::robot_interface {

template <typename T>
using Callback = std::function<void(T)>;

template <typename T>
class CallbackList {
   public:
    using CallbackPtr = std::weak_ptr<Callback<const T&>>;
    using CallbackPtrBase = std::shared_ptr<Callback<const T&>>;

    void operator()(const T& input) {
        auto i = callbacks.begin();

        while (i != callbacks.end()) {
            if (auto callback = (*i).lock()) {
                (*callback)(input);
                i++;
            } else {
                i = callbacks.erase(i);
            }
        }
    }

    void registerCallback(const CallbackPtrBase& ptr) { this->callbacks.emplace_back(ptr); }
    void deregisterCallback(const CallbackPtrBase& ptr) {
        auto i = callbacks.begin();

        while (i != callbacks.end()) {
            if ((*i).lock() != ptr) {
                i++;
            } else {
                i = callbacks.erase(i);
            }
        }
    }

   private:
    std::list<CallbackPtr> callbacks;
};

}  // namespace luhsoccer::robot_interface