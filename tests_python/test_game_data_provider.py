import baguette_py as baguette
import start_config


@start_config.event_based(start_config.setup_clean)
def test_game_data_provider(
    baguette_instance: baguette.Baguette, game_data_provider: baguette.GameDataProvider
):
    world_model = game_data_provider.getWorldModel()
    print(world_model)
