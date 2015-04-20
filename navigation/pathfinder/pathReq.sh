#!/bin/bash

## Script permettant d'appeler un service generatePath, générant un chemin sur la map de la Robocup LLSF 2014
# ATTENTION - l'id doit changer entre deux requêtes, une rêqûete de même id que la précédente sera ignorée
if [[ $# -ge 1 ]]; then
  id=$1
else
  id=0
fi

if [[ $# -eq 3 ]]; then
  x=$2
  y=$3
else
  x=2.0
  y=3.0
fi
echo "Path request #${id} for $x,$y"

rosservice call /generatePath "id: ${id}
Arrivee:
  position:
    x: $x
    y: $y
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
Depart:
  position:
    x: -5.0
    y: 3.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
utilisePositionOdometry: false"