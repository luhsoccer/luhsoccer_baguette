docker login registry.gitlab.com -u - -p glpat-3_T5PxDfDM9SZRRCMGsK
docker run -d --rm -t -i -p 127.0.0.1:10020:10020 -p 127.0.0.1:10300:10300 -p 127.0.0.1:10301:10301 -p 127.0.0.1:10302:10302 --name luhsoccer_er_sim registry.gitlab.com/luhbots/luhsoccer_baguette_er_sim:0.1.6 -g 2020B --realism RC2021 --feedback local-velocity
ECHO "Started er force simulation..."

docker attach --sig-proxy=false --detach-keys="ctrl-c" luhsoccer_er_sim

docker kill luhsoccer_er_sim
