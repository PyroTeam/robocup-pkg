Constructeur de la map
=================================

Nouvelle version restructurée mais fonctionnellement similaire à l'ancienne

Description
-----------
Ce package implémente le noeud de génération de la map à partir des caractéristiques du terrain et des élements le constituant (murs, machines...)

Noeuds
------
* `grid_maker_node` : noeud principal
* `fake_landmarks_node` : noeud fake, publiant des landmarks correspondant à des machines, utile pour tester des noeuds ne necessitant pas un simulateur et une localisation complète (ex. path_finder). Ce noeud prend en paramètre un fichier YAML contenant la liste des machines à publier

Launchfiles
-----------
* `grid_maker.launch` : lance le noeud `grid_maker_node` avec la map de la compétition
* `grid_maker_D336.launch` : Lance le noeud `grid_maker_node` avec la map de notre piste d'essai
* `grid_maker_debug.launch` : lance le noeud `grid_maker_node` en mode debug via le logiciel ddd

Fichiers de configurations
--------------------------
* `field.yaml` : contient les caractéristiques du terrain de la competition (dimension, murs, zones interdites, ...)
* `field_D336.yaml` : idem que `field.yaml`, mais pour la piste déssais en D336
* `landmarks.yaml` : contient une liste de machines avec leur pose, utilisé pour la publication de landmarks, pour le terrain de la compétition.
* `landmarks_D336.yaml` : idem que `landmarks.yaml` mais pour la piste d'éssais en D336
