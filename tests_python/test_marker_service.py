import baguette_py as baguette
import time
import start_config


@start_config.event_based(start_config.setup_clean)
def test_marker_service(
    baguette_instance: baguette.Baguette, marker_service: baguette.MarkerService
):
    cone_marker = baguette.TextMarker(baguette.Position(""), "cone", 1, "Some text")
    cone_marker.setColor(baguette.Color.red(1.0))
    cone_marker.setLifetime(10)

    marker_service.displayMarker(cone_marker)

    time.sleep(2)
