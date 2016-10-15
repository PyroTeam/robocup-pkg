# Localisation package

Ce package contient :

_extraction de landmarks :_
* détecte les droites dans un nuage de points (via [RANSAC](https://fr.wikipedia.org/wiki/RANSAC))
* détecte les segments à partir des modèles de droites trouvés par segmentation
* extrait les machines (segments de 70 cm)

_cartographie :_
* récupère les machines
* moyenne leur position en fonction des données reçues
* ajoute les machines mirrorées
* corrige l'angle via AR TAg
* associe position, IDs AR Tag et zone

_correction odométrie :_
* calcule et fusionne l'odométrie et les données gyroscopiques pour la vitesse angulaire

#### Paramètres


_RANSAC :_
* nuage de points
* nombre de points minimum pour considérer le modèle comme valide
* nombre de points pertinents dans le nuage
* probabilité de trouver un pattern connu dans le nuage de points
* seuil de proximité pour considérer qu'un point fait partie du modèle
* nombre de points minimum pour valider le modèle

_extraction des machines :_
* taille de la machine
* seuil d'acceptation


**DOIT être normalisé dans les common :**

* LaserScan (ou utilisation du pcl node)
* Line
* Segment
* Machine (comme objet physique, ce serait une classe mère pour les infos purement géométriques -> taille, position sur la piste)
* Subscriber AR TAgs ?
