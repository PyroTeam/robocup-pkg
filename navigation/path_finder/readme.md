Générateur de chemins
=================================

Package en cours de développement

Description
-----------
Ce package implémente la nouvelle version du path_finder.
La version actuelle utilise une OccupancyGrid avec l'algorithme Astar.

Noeuds
------
* `path_finder_node` : noeud principal

Launchfiles
-----------
* `path_finder_test_bench.launch` : lance le noeud `path_finder_node` ainsi que le `grid_maker` pour les tests
* `path_finder_ddd.launch` : Lance le noeud `path_finder_node`en mode debug via le logiciel ddd
