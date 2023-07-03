from tests_python.conftest import game_data_provider, baguette, baguette_instance
import time
import pytest


def test_game_data_provider(
    baguette_instance: baguette.Baguette, game_data_provider: baguette.GameDataProvider
) -> None:
    baguette_instance.start()

    world_model = game_data_provider.getWorldModel()
    print(game_data_provider.getWorldModel())
