#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define N 5
#define K 100
#define T 10
#define D 5

Modele ransac(laserScan::points pCloud)
{
  //initialisation du random (dans le main !!!)
  srand(time(NULL));

  int size = pCloud.size();

  //on copie le vector pCloud dans une liste
  std::list<Point> listOfPoints;
  for (int i = 0; i < size; i++){
    listOfPoints.push_back(pCloud[i]);
  }

  while (size < N){
    int iter = 0, i = 0;
    Modele meilleur_modele;

    while (iter < K){
      Modele modele_possible;
      Point a, b;

      //on prend deux points au hasard...
      i = rand() % listOfPoints.size();
      do
      {
        j = rand() % listOfPoints.size();
      }while(j==i);

      //on stocke les indices de ces points dans la liste d'indices du modele possible
      //ainsi que les points correspondants
      int cpt = 0, NbPtFound = 0;
      for (std::list<Point>::iterator it = listOfPoints.begin(); it != listOfPoints.end(); ++it){
        if (cpt == i){
          a = *it;
          modele_possible.addIndex(it);
          NbPtFound++;
        }
        if (cpt == j){
          b = *it;
          modele_possible.addIndex(it);
          NbPtFound++;
        }
        if (NbPtFound == 2){
          break;
        }
        cpt++;
      }

      //on fabrique alors une droite à partir de ces points
      modele_possible.build(a,b);
      
      //pour tous les autres points
      for (std::list<Point>::iterator it = listOfPoints.begin(); it != listOfPoints.end(); ++it){
        //si le point se situe dans le voisinage de la droite du modele_possible
        if (dist(*it, modele_possible.getDroite()) < T){
          //on ajoute l'indice correspondant à ce point dans la liste d'indices du modele_possible
          modele_possible.addIndex(it);
        }
      }

      //si le modele_possible contient assez de points
      if(modele_possible.getSize() > D){
        modele_possible.linReg();
      }

      if(modele_possible.getCorrel() < meilleur_modele.getCorrel()){
        meilleur_modele = modele_possible;
      }
    }
  }

  return meilleur_modele;
}