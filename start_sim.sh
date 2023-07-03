#!/bin/bash
DETACHED=false
REMOVE=false
CI=false
while getopts ":drc" opt; do
    case ${opt} in
         d) DETACHED=true;;
         r) REMOVE=true;;
         c) CI=true;;
    esac
done

if [ "$REMOVE" = true ] ; then
    docker kill luhsoccer_er_sim
    exit
fi

docker login registry.gitlab.com -u - -p glpat-3_T5PxDfDM9SZRRCMGsK
docker run -d --rm -t -i -p 127.0.0.1:10020:10020 -p 127.0.0.1:10300:10300 -p 127.0.0.1:10301:10301 -p 127.0.0.1:10302:10302 --name luhsoccer_er_sim registry.gitlab.com/luhbots/luhsoccer_baguette_er_sim:0.1.6 -g 2020B --realism RC2021 --feedback local-velocity
echo "Started er force simulation..."

if [ "$DETACHED" = false ] ; then

    function on_exit {
        docker kill luhsoccer_er_sim
    }
    trap on_exit EXIT

    docker attach --sig-proxy=false --detach-keys="ctrl-c" luhsoccer_er_sim
fi
