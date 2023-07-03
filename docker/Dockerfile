FROM frolvlad/alpine-glibc:alpine-3.16

LABEL maintainer="sebastian.knackstedt@luhbots.de"
LABEL version="0.0.6"
LABEL description="Custom docker image for ci checks and builds"

RUN apk add --no-cache git cmake pkgconfig make g++ python3-dev py3-pip cppcheck libice-dev libsm-dev libxaw-dev libxcb-dev libxcomposite-dev libxcursor-dev libxi-dev libxinerama-dev libxkbfile-dev libxrandr-dev libxres-dev libxscrnsaver-dev libxtst-dev libxv-dev libxvmc-dev xcb-util-wm-dev xcb-util-image-dev xcb-util-keysyms-dev xcb-util-renderutil-dev libfontenc-dev libxdamage-dev libxxf86vm-dev mesa-dev gcovr doxygen ccache bash
