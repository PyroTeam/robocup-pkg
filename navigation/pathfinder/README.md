pathfinder
==========

Description
-----------
Générateur de chemin

Noeuds
------
* `server` : Générateur de chemin avec algo AStar
* `server_static` : Générateur de chemin avec chemin codés en dur

rosrun pathfinder server  

Topics
------
* `pathFound` : Dernier chemin généré, avec id
* `path` : Dernier chemin généré au format nav_msgs::Path (pour rviz)
* `pathfinderState` : État du pathfinder

Services
--------
* `generatePath` : Demande la génération d'un chemin entre position Depart et Arrivee  
Peut utiliser la position odométrique comme position de départ avec le paramètre bolléen utiliser (à vérifier xD )

Launchfiles
-----------
* `pathfinder.launch` : lance le noeud pathfinder, ni plus ni moins

Scripts
-------
* `fakeGrid.sh` : Script permettant de générer un message nav_msgs/OccupancyGrid représentant la map de la Robocup LLSF 2014
* `pathReq.sh` :  Script permettant d'appeler un service generatePath, générant un chemin sur la map de la Robocup LLSF 2014  
ATTENTION - l'id doit changer entre deux requêtes, une rêqûete de même id que la précédente sera ignorée  
usage : pathReq.sh  
	pathReq.sh id  
	pathReq.sh id xArrivee yArrivee  
Par défaut, id=0, xArrivee=2, yArrivee=3 et (non dispo en argument) xDepart=-5, yDepart=-0.5  

Avancement
----------
* [x] Rédiger un README  
* [ ] Coder la nouvelle architecture
* [ ] Implémenter de nouveaux algorithmes
