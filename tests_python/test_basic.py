from tests_python.conftest import baguette_instance, baguette
import time
import pytest


def test_basic(baguette_instance: baguette.Baguette) -> None:
    baguette_instance.start()

    # Simple test to check if baguette is closing without error when using it over python
    time.sleep(5)

    # Double start should not do anything
    baguette_instance.start()
