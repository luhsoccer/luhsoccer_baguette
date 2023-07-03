#!/bin/bash
cwd=$(pwd)
export PYTHONPATH="$cwd/build/modules/python_bindings/:$PYTHONPATH"
export BAGUETTE_HEADLESS=1

cd stubs
/bin/bash generate_stubs.sh . || { echo "Test failed at generating stubs! Exiting"; exit 1; }
cd ..

pytest --junitxml=test_report.xml --cache-clear