#pragma once

#include <rigtorp/MPMCQueue.h>
#include <ssl_interface/ssl_types.hpp>
#include <robot_interface/events.hpp>
#include <condition_variable>
#include "vision_processor/vision_processor_events.hpp"

namespace luhsoccer::game_data_provider {

using Data = std::variant<ssl_interface::SSLFieldData, ssl_interface::SSLGameControllerData,
                          robot_interface::RobotFeedbackReceivedEvent, robot_interface::RobotCommandSendEvent,
                          vision_processor::ProcessedVisionData>;

class DataProcessor {
   public:
    template <typename Callable>
    void waitForNewData(Callable c) {
        {
            std::unique_lock lk(m);
            cv.wait_for(lk, std::chrono::milliseconds(500), [this] { return !data_queue.empty(); });
        }

        Data data;
        if (data_queue.try_pop(data)) {
            c(std::move(data));
        }
    }

    template <typename DataType>
    void appendData(DataType data) {
        if (!data_queue.try_emplace(std::forward<decltype(data)>(data))) {
            logger.warning("GameDataProvider Processing Queue is full");
        }
        cv.notify_one();
    }

   private:
    logger::Logger logger{"DataProcessor"};
    std::mutex m;
    std::condition_variable cv;
    rigtorp::MPMCQueue<Data> data_queue{64};
};

}  // namespace luhsoccer::game_data_provider
