approche_finale
===============

Description
-----------
* Package permettant d aborder les machines afin de pouvoir interagir avec elles
* L'approche finale se déroule en trois étapes :
    * Recherche d'un arTag correspondant à la demande de l'exécuteur de tâches
    * Asservissement grâce aux données de la caméra
    * Asservissement grâce aux données du laser

Noeuds
------
* `finalApproaching_node` : noeud principal 
* `fake_scan_node` : noeud de test pour valider le code autour de l action

Lignes de commandes
-------------------
*`rosrun approche_finale finalApproaching_node` ligne permettant de lancer l'approche finale
*`rosrun actionlib axclient.py /finalApproaching_node` ligne permettant de simuler un client

Champs possibles
----------------
*type : 0(BS) ou 1(RS) ou 2(CS) ou 3(DS)
*side : 100(in) 101(out)
*parameter : 10(S1) 20(S2) 30(S3) 40(LANE_RS) 50(LIGHT) 60(CONVEYOR)

Launchfiles
-----------
* `debug.launch` : lance le noeud finalApproaching_node avec ddd
* `approcheFinale.launch` : lance le noeud finalApproaching_node

Remarques utiles
----------------
* Pour l'instant, il est primordial de laisser les ROS_INFO dans le fichier `arTagFA.h` .  
Sans cela, il y a des erreurs de segmentation lors des appels aux fonctions qui y sont contenues.
* Bien mettre en paramètre la couleur de l'équipe (par défaut équipe cyan)
