
Entrée du programme :
 points – tableau de points initial issu des données lasers
Sortie du programme :
 modèles – tableau des meilleurs modèles trouvés par RANSAC

Entrées de l’algorithme de RANSAC :
 points – tableau de points 
 modele – une droite qui peut être ajusté à des points
 n - le nombre minimum de points nécessaires pour définir une droite
 k - le nombre maximal d'itérations de l'algorithme
 t - une valeur seuil pour déterminer si un point correspond à une droite
 d - le nombre de données proches des valeurs nécessaires pour faire valoir que la droite correspond bien aux points
Sorties de l’algorithme de RANSAC :
 meilleur_modèle - les paramètres de la droite qui correspondent le mieux aux points
 meilleur_ensemble_points - points à partir desquels la droite a été estimée
 meilleure_erreur - l'erreur du modèle de droite par rapport aux points
 tant qu’il reste au moins 5 points dans le tableau de points initial
  
  itérateur := 0
  meilleur_modèle := aucun
  meilleur_ensemble_points := aucun
  meilleure_erreur := infini
  tant que itérateur < k
    points_aléatoires := 2 points choisis au hasard à partir du tableau de points initial
    modèle_possible := paramètres de la droite correspondant aux points_aléatoires
    ensemble_points := points_aléatoires correspondant au modèle_possible
    Pour chaque point des données pas dans points_aléatoires
      si le point s'ajuste au modèle_possible avec une erreur inférieure à t
        Ajouter un point à ensemble_points
    si le nombre de points dans ensemble_points est > d
    (ce qui implique que nous avons peut-être trouvé un bon modèle,
     on teste maintenant dans quelle mesure il est correct)
      modèle_possible := régression linéaire de tous les points de ensemble_points
      erreur := √(∑▒〖erreur〗^2 )/(nombre de points)
    si erreur < meilleure_erreur
   (nous avons trouvé un modèle qui est mieux que tous les précédents,
    le garder jusqu'à ce qu'un meilleur soit trouvé)
      meilleur_modèle := modèle_possible
      meilleur_ensemble_points := ensemble_points
      meilleure_erreur := erreur
   stocker meilleur_modèle, meilleur_ensemble_points, meilleure_erreur dans tableau de modèles
   retirer meilleur_ensemble_points du tableau initial de points
   recommencer avec le tableau de points modifié