import baguette_py as baguette
import start_config


@start_config.event_based(start_config.setup_clean)
def test_time(baguette_instance: baguette.Baguette):
    now = baguette.now()
    print(f"{now}")
