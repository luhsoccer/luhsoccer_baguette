#include "baguette.hpp"
#include "core/events.hpp"
#include <csignal>
#include <luhsoccer-GitVersion.h>

namespace luhsoccer::baguette {

struct BuildInfo {
    unsigned int major = luhsoccer::version_major();
    unsigned int minor = luhsoccer::version_major();
    unsigned int patch = luhsoccer::version_major();
};

inline auto format_as(BuildInfo type) { return fmt::format("{}.{}.{},", type.major, type.minor, type.patch); }

void TheBaguette::load(bool install_signal_handler) {
    logger.info("Starting baguette version {}", luhsoccer::version_string());
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
        logger.debug("Running setup for module '{}'", module.get().moduleName());
        module.get().setup();
        module.get().setup(event_system);
    }
}

void TheBaguette::run() {
    should_run = true;
    // Start the timer events
    event_system.lockEventRegistration();
    event_system.startTimerEvents();
    event_system.startIOEvents();
    // Going through each module a start a new thread for each
    std::optional<std::reference_wrapper<BaguetteModule>> blocking_module = std::nullopt;
    for (const auto& module : modules) {
        if (module.get().is_blocking) {
            if (blocking_module.has_value()) {
                logger.warning("More then one blocking module defined. This should not happen.");
                break;
            }
            blocking_module = module.get();
            continue;
        }
        logger.debug("Starting module '{}'", module.get().moduleName());
        // Put the thread into a list of running thread to wait for the modules to finish
        running_threads.emplace_back([&module, this]() {
            // Loop the loop method
            while (this->should_run && module.get().is_running) {
                module.get().loop(should_run);
            }
            while (module.get().is_running) {
                std::this_thread::yield();
            }
            logger.debug("Loop of module '{}' has stopped", module.get().moduleName());
            return;
        });
        logger.debug("Module '{}' is now running in background", module.get().moduleName());
    }

    event_system.fireEvent(StartEvent());

    auto baguette_version = marker::Info("build_info", 0);
    std::string version(luhsoccer::version_string());
    baguette_version.set("Version", version);
    marker_service->displayMarker(std::move(baguette_version));

    auto baguette_branch = marker::Info("build_info", 1);
    std::string branch(luhsoccer::version_branch());
    baguette_branch.set("Branch", branch);
    marker_service->displayMarker(std::move(baguette_branch));

    auto baguette_commit = marker::Info("build_info", 2);
    std::string commit(luhsoccer::version_shorthash());
    baguette_commit.set("Commit", commit);
    marker_service->displayMarker(std::move(baguette_commit));

    // change something ss
    if (blocking_module.has_value()) {
        logger.debug("Main thread is running blocking module {}", blocking_module->get().moduleName());
        try {
            blocking_module->get().loop(this->should_run);
        } catch (std::exception& e) {
            logger.error("Blocking module {} threw an exception: {}", blocking_module->get().moduleName(), e.what());
            while (should_run) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            throw e;
        }
        logger.info("{} finished. Main thread now waiting for exit", blocking_module->get().moduleName());
        if (standalone) {
            this->signalHandler(0);
        } else {
            this->should_run = false;
            std::thread t([this]() { this->stop(); });
            t.detach();
        }
    }

    // Wait for every thread to finish before exiting this function
    for (auto& thread : running_threads) {
        if (thread.joinable()) {
            thread.join();
        } else {
            logger.warning("Found non-joinable thread!");
        }
    }
    logger.info("All threads have finished... Stopping");
    exited = true;
}

void TheBaguette::stop() {
    if (exited || stopping) {  // Already exited we don't need to stop anything
        return;
    }
    stopping = true;
    should_run = false;

    event_system.fireEvent(StopEvent());
    event_system.stopIOEvents();
    event_system.stopTimerEvents();

    // Calling the stop method of each module
    for (const auto& module : modules) {
        if (module.get().is_blocking) {
            continue;
        }
        logger.debug("Stopping module '{}'", module.get().moduleName());
        module.get().stop();
        module.get().is_running = false;
    }

    using namespace std::chrono_literals;

    // Give the main thread 500ms to exit. If that doesn't happen print a warning
    std::this_thread::sleep_for(2000ms);
    if (exited == false) {
        logger.warning("Program still running after 2s! Exiting in 1s...");
        std::this_thread::sleep_for(1000ms);
        // After another 1000ms just exit the complete process
        if (exited == false) {
            logger.error("Still no exit after 1s. Exiting now!");
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
            instance->logger.info("Received exit signal {}. Stopping modules", signal);
            instance->stop();
        });
        t.detach();
    }
}

}  // namespace luhsoccer::baguette