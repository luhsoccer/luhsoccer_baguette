import time
import baguette_py as baguette
import start_config


@start_config.event_based(start_config.setup_clean)
def test_basic(baguette_instance: baguette.Baguette) -> None:
    """
    This tests if baguette start and stops without any errors
    """
    time.sleep(5)


@start_config.event_based(start_config.setup_clean)
def test_get_instance(baguette_instance: baguette.Baguette) -> None:
    assert baguette.Baguette.getInstance() == baguette_instance
