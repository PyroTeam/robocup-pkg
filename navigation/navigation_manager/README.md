navigation_manager
==============
Package de supervision du déplacement

Description
-----------
Ce package permet la supervision du déplacement en reliant le Path Tracker et le Path Finder. On demande au générateur de chemin de trouver un chemin, puis lorsque ce dernier en a trouvé un, l'exécuteur de chemin est appelé afin de le suivre.
De plus, la détection d'obstacles (autre que les machines donc les robots ou les humains) est géré par ce package.

Noeuds
------
* navigation_manager_node : crée l'action qui permet à l'exécuteur de tâches de générer et exécuter un chemin suivant la position initiale du robot et la destination voulue

Launchfiles
-----------
* `nav_manager.launch` : lance le noeud navigation_manager_node

Scripts
-------

Avancement
----------
* [x] Rédiger un README
* [x] Relation entre générateur et exécuteur de chemin
* [ ] Détection d'obstacles :
	- [ ] Avec capteurs Sharp
	- [ ] Avec données du laser
