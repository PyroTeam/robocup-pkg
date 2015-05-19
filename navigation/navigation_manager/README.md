navigation_manager
==============
Package de supervision du déplacement

Description
-----------
Permet de réaliser la supervision du déplacement en demandant au Path Finder de trouver un chemin, et au Path Tracker de le suivre. Ce package gère également la détection d'obstacles autres que les machines (robots, humains).

Noeuds
------
* navigation_manager_node : crée l'action qui permet à l'exécuteur de tâches de lancer la supervision du déplacement

Launchfiles
-----------
* `nav_manager.launch` : lance le noeud navigation_manager_node

Scripts
-------

Avancement
----------
* [x] Rédiger un README
* [x] Supervision
* [ ] Détection d'obstacles
  - [ ] Avec les capteurs Sharp
  - [ ] Avec les données du laser
