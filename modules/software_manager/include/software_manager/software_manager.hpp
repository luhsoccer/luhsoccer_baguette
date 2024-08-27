#pragma once

#include <shared_mutex>
#include <unordered_map>

#include "core/module.hpp"
#include "logger/logger.hpp"
#include "time/time.hpp"

namespace luhsoccer::software_manager {

enum class SoftwareComponent { GAME_CONTROLLER, ER_SIM };

std::string_view format_as(SoftwareComponent component);

struct ComponentState {
    bool running = false;
    bool should_run = false;
    unsigned int pid = 0;
};

class NativeAddons;

class SoftwareManager : public BaguetteModule {
   public:
    SoftwareManager();
    ~SoftwareManager() override;

    SoftwareManager(const SoftwareManager&) = delete;
    SoftwareManager(SoftwareManager&&) = delete;
    SoftwareManager& operator=(const SoftwareManager&) = delete;
    SoftwareManager& operator=(SoftwareManager&&) = delete;

    void startComponent(SoftwareComponent component);

    std::vector<SoftwareComponent> getComponents() const;

    ComponentState getComponentState(SoftwareComponent component) const;

    void stopComponent(SoftwareComponent component);

    bool isRunning(SoftwareComponent component);

    bool shouldRun(SoftwareComponent component);

    constexpr std::string_view moduleName() override { return "software_manager"; }

    void setup(event_system::EventSystem& event_system) override;

    void update();

   private:
    bool hasComponent(SoftwareComponent component);
    void downloadComponent(SoftwareComponent component);
    void deleteComponent(SoftwareComponent component);
    unsigned int execFile(const std::string& file, const std::vector<std::string>& args);
    bool isAlive(unsigned int pid);
    bool stopSubprocess(unsigned int pid);
    bool download(const std::string& url, const std::string& file_name);

   private:
    mutable std::shared_mutex state_mutex;
    std::unordered_map<SoftwareComponent, ComponentState> component_states;
    std::unique_ptr<NativeAddons> native_addons;

    logger::Logger logger{"SoftwareManager"};
    time::TimePoint last_time;
};
}  // namespace luhsoccer::software_manager