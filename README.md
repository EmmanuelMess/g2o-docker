## Build

```
docker build --build-arg UID=$(id -u) -t g2otest .
```

## Running

```
docker run -it g2otest:latest bash
```

## Run g2o_viewer

```
xhost +
docker run -it --rm -it \
           -e DISPLAY=$DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
           -v $(pwd):/root/src:ro \
           g2otest:latest /root/g2o/bin/g2o_viewer

```