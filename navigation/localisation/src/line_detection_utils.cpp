#include "laserscan.h"
#include "Point.h"
#include "Droite.h"
#include "Modele.h"

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "line_detection_utils.h"

#define N 5
#define K 100
#define T 10
#define D 5

double dist(Point a, Droite d){
	Point b = d.getPoint();

	return abs(a.x*b.y - b.x*a.y) / sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}

Modele ransac(std::list<Point> listOfPoints)
{
  int size = pCloud.size();

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
      	//on fait une régression linéaire à partir des points appartenant au modèle
      	//pour ajuster les paramètres de la droite et calculer le coeff de corrélation
        modele_possible.linReg();
      }

      //si le modele_possible est mieux que le meilleur_modele enregistré
      if(modele_possible.getCorrel() < meilleur_modele.getCorrel()){
      	//on construit le nouveau meilleur_modele à partir du modele_possible
        meilleur_modele.constructFrom(modele_possible);
      }
    }
  }

  return meilleur_modele;
}

void maj(std::list<Point> &listWithoutPrecModelPoints, Modele m){
	std::list<std::list<Point>::iterator> indexes = m.getIndex();
	//pour tous les index contenus dans la liste d'index du modele
	for(std::list<std::list<Point>::iterator>::iterator it = m.begin();it != m.end();++it){
		//on supprime dans la liste le point correspondant à l'index enregistré dans le meilleur_modele
		listWithoutPrecModelPoints.erase(*it);
	}
}

std::list<Modele> findLines(std::list<Point> listOfPoints){
	std::list<Modele> listOfModeles;
	std::list<Point>  listWithoutPrecModelPoints = listOfPoints;
	Modele            m;

	while (listWithoutPrecModelPoints.size() > 5){
        m = ransac(listWithoutPrecModelPoints);
        maj(&listWithoutPrecModelPoints, m);
        listOfModeles.push_back(m);
    }

    return listOfModeles;
}