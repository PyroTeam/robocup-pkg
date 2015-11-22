Path_tracker
==============
Package de suivi de trajectoire et d'évitement d'obstacles

Description
-----------
Permet d'insérer les obstacles rencontrés par le robotino sur un grid map obstacle, de suivre le chemin généré par le pathfinder et d'éviter les éventuels obstacles intervenant sur la trajectoire suivie.

Noeuds
------
* `path_tracker_node` : lance le suivi de chemin/évitement d'obstacles
* `grid_obstacles_node` : récupère les données laser puis les insère dans la grid map obstacle

Launchfiles
-----------
* path_tracker.launch : lance les noeuds path_tracker_node et grid_obstacles_node
* robotCommddd.launch : lance les noeuds path_tracker_node et grid_obstacles_node en mode debug

-> Il faut au préalable que les launchfiles lançant les noeuds correspondant au pathfinder, à la navigation manager et au grid maker soient exécutés.

Scripts
-------

Avancement
----------
* [x] Rédiger un README
* [x] Suivi et évitement fonctionnels
