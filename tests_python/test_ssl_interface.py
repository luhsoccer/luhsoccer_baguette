from tests_python.conftest import baguette, baguette_instance, ssl_interface


def test_set_VisionDataSource(
    baguette_instance: baguette.Baguette, ssl_interface: baguette.SSLInterface
) -> None:
    vision_source = baguette.VisionDataSource.Simulation

    ssl_interface.setVisionDataSource(vision_source)

    assert ssl_interface.getVisionDataSource() == vision_source
