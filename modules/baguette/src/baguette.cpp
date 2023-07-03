#include "baguette.hpp"
#include "build-info.hpp"
#include <csignal>

namespace luhsoccer::baguette {

struct BuildInfo {
    unsigned int major = PROJECT_VERSION_MAJOR;
    unsigned int minor = PROJECT_VERSION_MINOR;
    unsigned int patch = PROJECT_VERSION_PATCH;
};

std::ostream& operator<<(std::ostream& os, const BuildInfo& c) {
    return os << c.major << "." << c.minor << "." << c.patch;
}

void TheBaguette::load(bool install_signal_handler) {
    LOG_INFO(logger, "Starting baguette version {}", BuildInfo{});
    started = true;
    exited = false;

    // Install a SIGINT handler if desired
    if (install_signal_handler) {
        std::signal(SIGINT, signalHandler);
    } else {
        standalone = false;
    }

    // Running the setup method of each module
    for (const auto& module : modules) {
        LOG_DEBUG(logger, "Running setup for module '{}'", module.get().moduleName());
        module.get().setup();
    }
}

void TheBaguette::run() {
    should_run = true;
    // Going through each module a start a new thread for each
    std::optional<std::reference_wrapper<BaguetteModule>> blocking_module = std::nullopt;
    for (const auto& module : modules) {
        if (module.get().is_blocking) {
            if (blocking_module.has_value()) {
                LOG_WARNING(logger, "More then one blocking module defined. This should not happen.");
                break;
            }
            blocking_module = module.get();
            continue;
        }
        LOG_DEBUG(logger, "Starting module '{}'", module.get().moduleName());
        // Put the thread into a list of running thread to wait for the modules to finish
        running_threads.emplace_back([&module, this]() {
            // Loop the loop method
            while (this->should_run) {
                module.get().loop(should_run);
            }
            while (module.get().is_running) {
                std::this_thread::yield();
            }
            LOG_DEBUG(logger, "Module '{}' has stopped", module.get().moduleName());
            return;
        });
        LOG_DEBUG(logger, "Module '{}' is now running in background", module.get().moduleName());
    }
    // change something ss
    if (blocking_module.has_value()) {
        LOG_DEBUG(logger, "Main thread is running blocking module {}", blocking_module->get().moduleName());
        try {
            blocking_module->get().loop(this->should_run);
        } catch (std::exception& e) {
            LOG_ERROR(logger, "Blocking module {} threw an exception: {}", blocking_module->get().moduleName(),
                      e.what());
            while (should_run) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        LOG_INFO(logger, "{} finished. Main thread now waiting for exit", blocking_module->get().moduleName());
        if (standalone) {
            this->signalHandler(0);
        } else {
            this->should_run = false;
        }
    }

    // Wait for every thread to finish before exiting this function
    for (auto& thread : running_threads) {
        if (thread.joinable()) {
            thread.join();
        } else {
            LOG_WARNING(logger, "Found non-joinable thread!");
        }
    }
    LOG_INFO(logger, "All threads have finished... Stopping");
    exited = true;
}

void TheBaguette::stop() {
    if (exited || stopping) {  // Already exited we don't need to stop anything
        return;
    }
    stopping = true;
    should_run = false;

    // Calling the stop method of each module
    for (const auto& module : modules) {
        if (module.get().is_blocking) {
            continue;
        }
        LOG_DEBUG(logger, "Stopping module '{}'", module.get().moduleName());
        module.get().stop();
        module.get().is_running = false;
    }

    using namespace std::chrono_literals;

    // Give the main thread 500ms to exit. If that doesn't happen print a warning
    std::this_thread::sleep_for(2000ms);
    if (exited == false) {
        LOG_WARNING(logger, "Program still running after 2s! Exiting in 1s...");
        std::this_thread::sleep_for(1000ms);
        // After another 1000ms just exit the complete process
        if (exited == false) {
            LOG_ERROR(logger, "Still no exit after 1s. Exiting now!");
            std::exit(-1);
        }
    }
    stopping = false;
}

// the same reason as defined in the header
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::vector<TheBaguette*> TheBaguette::instances;

void TheBaguette::signalHandler(int signal) {
    // Looping through every instance of the software and send exit stop command
    for (const auto& instance : TheBaguette::instances) {
        std::thread t([&instance, signal]() {
            LOG_INFO(instance->logger, "Received exit signal {}. Stopping modules", signal);
            instance->stop();
        });
        t.detach();
    }
}

}  // namespace luhsoccer::baguette