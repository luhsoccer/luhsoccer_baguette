import baguette_py as baguette
import start_config


@start_config.event_based(start_config.setup_clean)
def test_set_vision_data_source(
    baguette_instance: baguette.Baguette, ssl_interface: baguette.SSLInterface
) -> None:
    vision_source = baguette.VisionDataSource.Simulation

    ssl_interface.setVisionDataSource(vision_source)

    assert ssl_interface.getVisionDataSource() == vision_source
