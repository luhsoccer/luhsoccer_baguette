#include <iostream>
#include <memory>

namespace luhsoccer::game_controller_interface {

class GameControllerInterfaceImpl;

class GameControllerInterface {
   public:
    GameControllerInterface(std::string gc_path);
    GameControllerInterface(const GameControllerInterface&) = delete;
    GameControllerInterface(GameControllerInterface&&) = delete;
    GameControllerInterface& operator=(const GameControllerInterface&) = delete;
    GameControllerInterface& operator=(GameControllerInterface&&) = delete;
    ~GameControllerInterface();

    /**
     * @brief Starts a game controller
     *
     * @return true The Game Controller started successfully
     * @return false The Game Controller could not be started
     */
    bool startGameController();

    /**
     * @brief Stops the Game Controller associated with this Interface
     *
     * @return true The Game Controller terminated successfully
     * @return false The Game Controller could not be terminated (it might still be running in the background)
     */
    bool stopGameController();

   private:  // Methods
   private:  // Member variables
    std::unique_ptr<GameControllerInterfaceImpl> impl{nullptr};
};

}  // namespace luhsoccer::game_controller_interface