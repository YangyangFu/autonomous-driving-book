docker run^
    -e SDL_VIDEODRIVER=x11 ^
    -e DISPLAY=$DISPLAY ^
    -e XAUTHORITY=$XAUTHORITY ^
    -v /tmp/.X11-unix:/tmp/.X11-unix ^
    -v $XAUTHORITY:$XAUTHORITY ^
    -it --gpus 'all,"capabilities=graphics,utility,display,video,compute"' ^
    -p 2000-2002:2000-2002 carlasim/carla:0.9.14 ^
    ./CarlaUE4.sh -quality-level=Low -fps=15 -windowed -ResX=600 -ResY=480