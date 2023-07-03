#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
GC_FOLDER=$SCRIPT_DIR/runtime/gc
GC_BIN=$GC_FOLDER/ssl-game-controller_v*

if [ ! -d "$GC_FOLDER" ]; then
    mkdir -p $GC_FOLDER
fi

if  ls $GC_BIN 1> /dev/null 2>&1; then
    echo "GC Binary already downloaded."
else
    cd $GC_FOLDER
    curl -s https://api.github.com/repos/RoboCup-SSL/ssl-game-controller/releases/latest \
    | grep "ssl-game-controller_.*_linux_amd64" \
    | cut -d : -f 2,3 \
    | tr -d \" \
    | wget -qi -
    cd -
    chmod +x $GC_BIN
fi


cd $GC_FOLDER
$GC_BIN -timeAcquisitionMode ci -publishAddress 0
cd -