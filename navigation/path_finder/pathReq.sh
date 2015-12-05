#!/bin/bash

## Script permettant d'appeler un service generatePath, générant un chemin sur la map de la Robocup LLSF 2014
# ATTENTION - l'id doit changer entre deux requêtes, une rêqûete de même id que la précédente sera ignorée
if [[ $# -ge 1 ]]; then
  id=$1
else
  id=0
fi

if [[ $# -eq 5 ]]; then
  xd=$2
  yd=$3
  xa=$4
  ya=$5
else
  xd=0.0
  yd=0.0
  xa=-2.0
  ya=3.0
fi
echo "Path request #${id} for $xd,$yd to $xa,$ya"

rosservice call /generatePath "id: ${id}
Arrivee:
  position:
    x: $xa
    y: $ya
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
Depart:
  position:
    x: $xd
    y: $yd
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
utilisePositionOdometry: false"
