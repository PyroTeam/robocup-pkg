#include "laserScan.h"
#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Segment.h"

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <list>

#include "line_detection_utils.h"

deplacement_msg::Point convertPtToDeplMsgPt(const Point &point){
	deplacement_msg::Point p;
	p.x = point.getX();
	p.y = point.getY();

	return p;
}

deplacement_msg::Points convertPtsToDeplMsgPts(const std::list<Point> &points){
    deplacement_msg::Points listOfDeplMsgPt;
    listOfDeplMsgPt.points.resize(points.size());

    for (std::list<Point>::const_iterator it = points.begin(); it != points.end(); ++it){
        deplacement_msg::Point pt = convertPtToDeplMsgPt(*it);
        listOfDeplMsgPt.points.push_back(pt);
    }

    return listOfDeplMsgPt;
}

deplacement_msg::Droite convertMdlToDeplMsgDroite(const Modele &modele){
	//on récupère les infos nécessaires du modele
	Droite d     = modele.getDroite();
  Point p      = d.getPoint();
  double angle = d.getAngle();

  deplacement_msg::Droite msgDroite;
  msgDroite.point = convertPtToDeplMsgPt(p);
  msgDroite.angle = angle;

  return msgDroite;
}

deplacement_msg::Droites convertModelesToDeplMsgDroites(const std::list<Modele> &modeles){
    deplacement_msg::Droites droites;
    droites.droites.resize(modeles.size());

    for (std::list<Modele>::const_iterator it = modeles.begin(); it != modeles.end(); ++it){
    	deplacement_msg::Droite d = convertMdlToDeplMsgDroite(*it);
      droites.droites.push_back(d);
    }

    return droites;
}

double dist(Point a, Droite d){
	Point b = d.getPoint();
  Point u(cos(d.getAngle()), sin(d.getAngle()));
  Point ba(a.getX()-b.getX(), a.getY()-b.getY());

	return std::abs(u.getX()*ba.getY() - ba.getX()*u.getY()) / sqrt(u.getX()*u.getX() + u.getY()*u.getY());
}

Point ortho(Point a, Droite d){
  double distance = dist(a,d);
  double dx = distance*cos(d.getAngle());
  double dy = distance*sin(d.getAngle());
  Point p(a.getX() - dx, a.getY() + dy);

  return p;
}

Modele ransac(std::list<Point> &listOfPoints, int n, int NbPtPertinent, double proba, double seuil, int NbPts)
{
  int size = listOfPoints.size();
  printf("size of ListOfPoints = %d\n", size);
  int iter = 0, i = 0, j = 0;
  double w = double(NbPtPertinent)/double(size);
  printf("w = %f\n", w);
  double k = log10(1-proba)/log10(1-pow(w,n));
  printf("k = %f\n", k);
  Modele meilleur_modele;

  while (iter < k){
    Modele modele_possible;
    Point a, b;
    //on prend deux points au hasard
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
        NbPtFound++;
      }
      if (cpt == j){
        b = *it;
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
      if (dist(*it, modele_possible.getDroite()) < seuil){
        //on ajoute l'indice correspondant à ce point dans la liste d'indices du modele_possible
        modele_possible.addIndex(it);
      }
    }

    //si le modele_possible contient assez de points
    if(modele_possible.getIndex().size() > NbPts){
    	//on fait une régression linéaire à partir des points appartenant au modèle
    	//pour ajuster les paramètres de la droite et calculer le coeff de corrélation
      modele_possible.linReg();
    }

    //si le modele_possible est mieux que le meilleur_modele enregistré
    if (modele_possible.getIndex().size() > meilleur_modele.getIndex().size()){
    	//on construit le nouveau meilleur_modele à partir du modele_possible
      meilleur_modele.constructFrom(modele_possible);
      //printf("correlation : %f\n", meilleur_modele.getCorrel());
    }

    iter++;
  }

  return meilleur_modele;
}

void maj(std::list<Point> &list, Modele m){
	const std::list<std::list<Point>::iterator> &indexes = m.getIndex();
	//pour tous les index contenus dans la liste d'index du modele
	for(std::list<std::list<Point>::iterator>::const_iterator it = indexes.cbegin(); it != indexes.cend(); ++it){
		//on supprime dans la liste le point correspondant à l'index enregistré dans le meilleur_modele
		//printf("%f \t %f\n", (*it)->getX(), (*it)->getY());
    list.erase(*it);
    //printf("taille de la liste %d\n", list.size());
	}
}

std::list<Modele> findLines(std::list<Point> &listOfPoints){
	std::list<Modele> listOfDroites;
	std::list<Point>  listWithoutPrecModelPoints = listOfPoints;
	Modele            m;
  bool              stopRansac = false;

	while (!stopRansac){
        //printf("entrée findlines\n");
        //ransac(std::list<Point> &listOfPoints, int n, int NbPtPertinent, double proba, double seuil, int NbPts)
        int taille = listWithoutPrecModelPoints.size();
        m = ransac(listWithoutPrecModelPoints, 2, 20, 0.95, 0.05, 20);
        if( std::abs(m.getCorrel()) > 0.01)
        {
          //printf("taille de la droite %d\n", m.getIndex().size());
          maj(listWithoutPrecModelPoints, m);
          //printf("taille de la liste %d\n", listWithoutPrecModelPoints.size());
          listOfDroites.push_back(m);
        }
        else
        {
          stopRansac = true;
        }
    }

    return listOfDroites;
}

