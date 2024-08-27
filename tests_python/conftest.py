import pytest
import baguette_py as baguette


@pytest.fixture
def baguette_instance():
    instance = baguette.Baguette.getInstance()
    yield instance

    if not instance._started:
        print(
            "Baguette instance was not started. Perhaps you forgot use the @start_config decorator."
        )

    assert instance._started


@pytest.fixture
def ssl_interface(baguette_instance: baguette.Baguette) -> baguette.SSLInterface:
    return baguette_instance.ssl_interface


@pytest.fixture
def marker_service(baguette_instance: baguette.Baguette) -> baguette.MarkerService:
    return baguette_instance.marker_service


@pytest.fixture
def game_data_provider(
    baguette_instance: baguette.Baguette,
) -> baguette.GameDataProvider:
    return baguette_instance.game_data_provider


@pytest.fixture
def skill_library(baguette_instance: baguette.Baguette) -> baguette.SkillLibrary:
    return baguette_instance.skill_library


@pytest.fixture
def skill_book(baguette_instance: baguette.Baguette) -> baguette.SkillBook:
    return baguette_instance.skill_book
