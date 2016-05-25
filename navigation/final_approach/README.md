FinalApproach
=============

Description
-----------
* Package permettant d'aborder les machines afin de pouvoir interagir avec elles par la suite
* L'approche finale se déroule actuellement en trois étapes :
    * Recherche d'un arTag correspondant à la demande de l'exécuteur de tâches
    * Asservissement grâce aux données de la caméra, approche à 50cm
    * Asservissement grâce aux données du laser
    	1. En ciblant en Y d'abord
    	2. En avancant tout droit en suite

Noeuds
------
* `finalApproaching_node` : noeud principal
* `fake_scan_node` : noeud de test pour valider le code autour de l action

Lignes de commandes
-------------------
* `rosrun final_approach finalApproaching_node` : ligne permettant de lancer l'approche finale
* `rosrun actionlib axclient.py /finalApproaching_node` : ligne permettant de simuler un client

Champs possibles
----------------
* type : 0(BS) ou 1(RS) ou 2(CS) ou 3(DS)
* side : 100(in) 101(out)
* parameter : 10(S1) 20(S2) 30(S3) 40(LANE_RS) 50(LIGHT) 60(CONVEYOR)

Launchfiles
-----------
* `debug.launch` : lance le noeud finalApproaching_node avec ddd
* `approcheFinale.launch` : lance le noeud finalApproaching_node
* `test_finalApproach.launch` : lance le noeud finalApproaching_node avec du ROS_DEBUG et rviz. Destiné à être lancé
avec gazebo_sim_launch/launch/env_finalApproach.launch

Remarques utiles
----------------
* Pour l'instant, il est primordial de laisser les ROS_INFO dans le fichier `arTagFA.h` .
Sans cela, il y a des erreurs de segmentation lors des appels aux fonctions qui y sont contenues.
* Bien mettre en paramètre la couleur de l'équipe (par défaut équipe cyan)

Travail restant
---------------
* [x] Collecter les infos équipe et phase du topic dans le package refBox_comm
* [x] Faire le test ROS_UNLIKELY(true) pour chercher à résoudre l'erreur de segmentation
* [ ] Gestion des murs qui sont à proximité des machines
* [ ] Déterminer au préalable si un objet existe dans la zone à explorer

----

## BILAN
Je vais résumer ici les problèmes connus, améliorations possibles, etc

### Problèmes connus :
 * L'utilisation des ArTags est limitée au repère de la caméra, il n'y a pas de changement de repère effectué.
 * L'asservissement laser est particulièrement lent.
 * Le filtre sur les machines "accessible" n'est pas du ressort de l'approche finale.
 * Il faut reprendre tous les launchfiles pour s'assurer que ar_track_alvar publie dans le bon repère à chaque fois.
 * Le launchfile robotino_node doit être repris.

### Conditions nécéssaire :
 * [ ] Arttrack_alvar doit publier ses positions dans le repère caméra.
 * [ ] Le robot doit être à moins de 2m de la machine (filtrage des arTag trop éloignés)

### Améliorations possibles :
 * Make a common lib with PID or look after http://wiki.ros.org/pid and http://wiki.ros.org/lyap_control
 * Paramétrer les asservissements
   * Paramètrages PID
   * Paramétrages limites (recherche d'arTag par exemple)
   * Paramétrages des positions intermédiaires
 * Moyenner les segments dans le repère map, pondérés par le coefficient de correllation
 * Filtrer les segments autour de la position détectée de l'arTag (plus fiable que la simple distance)