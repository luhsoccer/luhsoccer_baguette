#!/bin/bash
echo "Generating stubs for module _bagutte_py. Path: $1"
cd $1
export PYTHONPATH=".:$PYTHONPATH"
# Generate the basic stubs
pybind11-stubgen _baguette_py || { echo "Error while generating stubs"; exit 1; }
# Copy the right stubs file
cp stubs/_baguette_py-stubs/__init__.pyi _baguette_py.pyi
# Remove all other files
rm -r stubs
# Patch the stubs to have the right module name (tested with pylance)
sed -i 's/_baguette_py.//g' _baguette_py.pyi
# Create a typing file
touch py.typed