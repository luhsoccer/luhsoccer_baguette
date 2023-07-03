#!/bin/bash

cd wheelhouse
ls

# python3 -m pip install wheel --upgrade

for file in *; do
    name=$(echo $file | cut -d - -f 1)
    version=$(echo $file | cut -d - -f 2)
    folder="$name-$version"
    python3 -m wheel unpack $file
    echo "Patching $file"
    ##### Modify wheel
    cp ../stubs/_baguette_py.pyi $folder/baguette_py/_baguette_py.pyi
    touch $folder/baguette_py/py.typed
    #####
    python3 -m wheel pack $folder
    rm -r $folder
done

# For some reason the duplicated wheel doesn't contains types information. Otherwise they are identical so we just delete the wrongs ones
find . -name '*.manylinux_2_27*' -delete

ls
cd ..