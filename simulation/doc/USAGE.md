Environnement de simulation sur gazebo - UTILISATION
====================================================

Principe et lancement de la simulation
--------------------------------------
Globalement, pour lancer la simulation de la Robocup Logistic League, il faut démarrer gazebo avec un monde (un environnement) contenant les modèles et utilisant les plugins de `gazebo_rcll`. Par exemple le monde [field2015_sixRobots.world](../gazebo_sim_launch/worlds/field2015_sixRobots.world).  

Pour faire la jonction entre les topics gazebo utilisés par la simulation et nos topics ROS, il faut démarrer le noeud [`gazeo_to_ros`](../gazebo_to_ros).  
Certains noeuds ont un équivalent dédié à la simulation, c'est le cas par exemple du traitement d'images.  

Pour simplifier l'usage, on peut s'appuyer sur des launchfiles complets comme [env_global.launch](../gazebo_sim_launch/launch/env_global.launch).  
Exemple:
```Shell
roslaunch gazebo_sim_launch env_global.launch sim:=true
```

Utilisation après lancement
---------------------------
Dans les environnements de simulation complets, un plugin `mps_placement` est en charge de placer les machines MPS de la robocup sur le terrain, en conformité avec leur placement dans la referee box. En conséquence, la referee box doit être lancé pour que la simulation fonctionne.  
`mps_placement` ne place pas les machines si la referee box est en état `PRE_GAME` et si la simulation n'a pas dépassé les 15 secondes d'éxecution. Le chargement des MPS est long. Il est conseillé de passer la refbox en phase de `SETUP` jusqu'à ce que la simulation soit entièrement chargée.  