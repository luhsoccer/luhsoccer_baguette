
echo "Make Sure you configured the project beforehand!"
if [[ $1 != "stubs" ]] 
then 
    echo "Set first argument to 'stubs' to generate stubs as well"
fi
read -p "Press ENTER to continue"

pip uninstall pytest -y 

if [[ $1 == "stubs" ]] 
then 
    echo "tmp"    
    ninja -C build 

    CMAKE_GENERATOR=Ninja pip wheel . -vvv

    ./run_python_tests.sh

    mkdir wheelhouse

    rm wheelhouse/*

    mv luhsoccer_baguette-* wheelhouse

    ./patch_wheels.sh

    pip install --force-reinstall wheelhouse/luhsoccer_baguette-*

else 
    echo "add argument 'stubs' to generate stubs as well"

    CMAKE_GENERATOR=Ninja pip wheel . -vvv

    ./run_python_tests.sh

    pip install --force-reinstall luhsoccer_baguette-*
fi 


