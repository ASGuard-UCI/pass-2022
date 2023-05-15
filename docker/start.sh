#!/bin/sh

plugin=shapeshifter

echo "select the operation [1/2/3/4]:"
echo "  1) ShapeShifter attack"
echo "  2) RP2 attack"

read n
case $n in
  1) echo "You chose ShapeShifter attack"; plugin=shapeshifter;;
  2) echo "You chose RP2 attack"; plugin=rp2;;
  *) echo "invalid option"; exit;;
esac

DOCKER_MAJOR_VERSION=$(docker version --format '{{.Client.Version}}' | cut -d. -f1)
if [ $DOCKER_MAJOR_VERSION -ge 19 ] && ! which nvidia-docker > /dev/null; then
     readonly RUNTIME="--gpus=all"
else
     readonly RUNTIME="--runtime=nvidia"
fi

PWD=`pwd`

XAUTH=/home/$USER/.Xauthority
TMP=/tmp

usr="--user $(id -u $USER):$(id -g $USER)"
mountdirs="-v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro"
vimdirs="-v /home/$USER/.vim:/home/$USER/.vim:ro -v /home/$USER/.vimrc:/home/$USER/.vimrc:ro"

docker run \
     -it --rm -d \
     --privileged \
     $RUNTIME \
     ${usr} \
     ${mountdirs} \
     ${vimdirs} \
     --net=host \
     --volume=$TMP:$TMP:rw \
     --volume=$XAUTH:$XAUTH:rw \
     --env="XAUTHORITY=${XAUTH}" \
     -v ${PWD}:${PWD} \
     -w /home/$USER \
     --name=achilles-${plugin} \
     evisie/achilles-${plugin}
