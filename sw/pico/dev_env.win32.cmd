set CURRENTDIR=%cd%
docker run -d -it --name pico-sdk-fabricbuild -v %CURRENTDIR%:/home/dev lukstep/raspberry-pi-pico-sdk:latest
docker exec -it pico-sdk-fabricbuild /bin/sh
PAUSE