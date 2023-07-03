#pragma once

#include <string_view>
#include <atomic>
#include <thread>

namespace luhsoccer::baguette {
class TheBaguette;
}

namespace luhsoccer {

/**
 * @brief This class represents a concrete application module
 * The constructor of a module should not do any heavy work besides initializing the components. Everything else (IO or
 * computations) that needs to happen a single time should happen in the setup function of the module
 */
class BaguetteModule {
    friend class baguette::TheBaguette;

   private:
    std::atomic_bool is_running = true;
    bool is_blocking = false;

   public:
    /**
     * @brief Will be executed a single time.
     *
     */
    virtual void setup(){};
    /**
     * @brief This function will be called in a loop until the stop of the module
     * You can implement your own while loop using the should run variable.
     * @param should_run use is for your own loop
     */
    virtual void loop([[maybe_unused]] std::atomic_bool& should_run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    };
    /**
     * @brief This is called when the module should be stopped. This is not necessary to implement, but could be useful
     * if the module started their own custom threads that needs to stop.
     * The stopping of the module should happen immediately. The main threads gives currently a wait time of 500ms
     * before printing a warning and another 1000ms before killing thread.
     */
    virtual void stop(){};

    /**
     * @brief Return the name of the module. Module names should be in the same format as the header file for example
     * "my_custom_module".
     *
     * @return std::string_view
     */
    virtual constexpr std::string_view moduleName() = 0;
};

}  // namespace luhsoccer
