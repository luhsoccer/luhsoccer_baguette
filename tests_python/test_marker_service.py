from tests_python.conftest import marker_service, baguette_instance, baguette
import pytest
import time


def test_marker_service(
    baguette_instance: baguette.Baguette, marker_service: baguette.MarkerService
) -> None:
    baguette_instance.start()

    cone_marker = baguette.TextMarker(baguette.Position(""), "cone", 1, "Some text")
    cone_marker.setColor(baguette.Color.red(1.0))
    cone_marker.setLifetime(10)

    marker_service.displayMarker(cone_marker)

    time.sleep(2)
