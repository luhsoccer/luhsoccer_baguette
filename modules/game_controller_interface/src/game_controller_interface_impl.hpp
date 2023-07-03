#include <iostream>
#include <memory>
#include "logger/logger.hpp"
#include "process.hpp"

namespace luhsoccer::game_controller_interface {

class GameControllerInterfaceImpl {
   public:
    GameControllerInterfaceImpl(std::string gc_path);
    GameControllerInterfaceImpl(const GameControllerInterfaceImpl&) = delete;
    GameControllerInterfaceImpl(GameControllerInterfaceImpl&&) = delete;
    GameControllerInterfaceImpl& operator=(const GameControllerInterfaceImpl&) = delete;
    GameControllerInterfaceImpl& operator=(GameControllerInterfaceImpl&&) = delete;
    ~GameControllerInterfaceImpl();

    bool startGameController();
    bool stopGameController();

   private:  // Methods
   private:  // Member variables
    std::string path_to_gc;
    logger::Logger gc_logger;
    std::optional<TinyProcessLib::Process> gc_proc = std::nullopt;
};

}  // namespace luhsoccer::game_controller_interface