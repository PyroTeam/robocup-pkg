generateur_taches
=================

Description
-----------
Package contenant le générateur de tâches, ie le package qui commande et qui ordonnance
les différentes tâches à réaliser

Noeuds
------
* `main_node` : 			 noeud principal du générateur de tâches
* `action_node` : 			 noeud de test pour recevoir les infos du topic manager_msg::activity
* `activityTalker_node` : 	 noeud de test pour simuler l'envoi du topic manager_msg::activity
* `orderListener_node` : 	 noeud de test du serveur pour le service manager_msg::order
* `sendOrder_node` : 		 noeud de test pour simuler un client du service manager_msg::order
* `workFonctionsTest_node` : noeud pour tester les fonctions de work.cpp

Launchfiles
-----------
* `debug.launch` : 			 lance le noeud main_node avec ddd
* `tasksGenerator.launch` :  lance le noeud main_node
