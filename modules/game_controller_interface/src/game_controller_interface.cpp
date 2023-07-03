#include "game_controller_interface/game_controller_interface.hpp"
#include "game_controller_interface_impl.hpp"
#include "process.hpp"

namespace luhsoccer::game_controller_interface {

GameControllerInterface::GameControllerInterface(std::string gc_path)
    : impl(std::make_unique<GameControllerInterfaceImpl>(std::move(gc_path))){};

// The destructor can only be defined here because the complete type of GameControllerInterfaceImpl is first known here
// NOLINTNEXTLINE(performance-trivially-destructible)
GameControllerInterface::~GameControllerInterface() = default;

bool GameControllerInterface::startGameController() { return this->impl->startGameController(); }

bool GameControllerInterface::stopGameController() { return this->impl->stopGameController(); }

}  // namespace luhsoccer::game_controller_interface