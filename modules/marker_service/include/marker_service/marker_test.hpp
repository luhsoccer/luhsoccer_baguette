#pragma once

#include "game_data_provider/game_data_provider.hpp"
#include "marker_service/marker_service.hpp"

namespace luhsoccer::marker {

class MarkerTest {
   public:
    MarkerTest(MarkerService& ms, game_data_provider::GameDataProvider& gdp) : gdp(gdp), ms(ms) {}
    void displayTestMarkers();

    int random(int min, int max);

   private:
    game_data_provider::GameDataProvider& gdp;
    MarkerService& ms;
};

}  // namespace luhsoccer::marker