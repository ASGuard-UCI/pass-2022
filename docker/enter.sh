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

uid=`id -u $USER`
docker exec -u $uid -it achilles-${plugin} bash
