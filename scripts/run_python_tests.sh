#!/bin/bash
cwd=$(pwd)
export PYTHONPATH="$cwd/build/modules/python_bindings/:$PYTHONPATH"
export BAGUETTE_HEADLESS=1

pytest --junitxml=test_report.xml --cache-clear