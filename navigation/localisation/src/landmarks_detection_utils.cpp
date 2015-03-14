#include "laserScan.h"
#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Segment.h"
#include "Machine.h"
#include "sensor_msgs/LaserScan.h"

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <list>
#include <algorithm>
#include <iterator>
#include <limits>

#include "landmarks_detection_utils.h"

double dist(Point a, Droite d){
	Point b = d.getPoint();
  Point u(cos(d.getAngle()), sin(d.getAngle()));
  Point ba(a.getX()-b.getX(), a.getY()-b.getY());

	return std::abs(u.getX()*ba.getY() - ba.getX()*u.getY()) / sqrt(u.getX()*u.getX() + u.getY()*u.getY());
}

double dist(Point a, Segment s){
  Point b = s.getMin();
    Point u(cos(s.getAngle()), sin(s.getAngle()));
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

Point ortho(Point a, Segment s){
  double distance = dist(a,s);
  double dx = distance*cos(s.getAngle());
  double dy = distance*sin(s.getAngle());
  Point p(a.getX() - dx, a.getY() + dy);

  return p;
}

Modele ransac(std::list<Point> &listOfPoints, int n, int NbPtPertinent, double proba, double seuil, int NbPts){
  int size = listOfPoints.size();
  int iter = 0, i = 0, j = 0;
  double w = double(NbPtPertinent)/double(size);
  double k = log10(1-proba)/log10(1-pow(w,n));

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
    //if (modele_possible.getCorrel() > meilleur_modele.getCorrel()){
    	//on construit le nouveau meilleur_modele à partir du modele_possible
      meilleur_modele = modele_possible;
    }

    iter++;
  }
  meilleur_modele.update();

  return meilleur_modele;
}

void maj(std::list<Point> &list, Modele m){
	const std::list<std::list<Point>::iterator> &indexes = m.getIndex();
	//pour tous les index contenus dans la liste d'index du modele
	for(std::list<std::list<Point>::iterator>::const_iterator it = indexes.cbegin(); it != indexes.cend(); ++it){
		//on supprime dans la liste le point correspondant à l'index enregistré dans le meilleur_modele
		//printf("%f \t %f\n", (*it)->getX(), (*it)->getY());
    list.erase(*it);
	}
}

std::list<Modele> findLines(const std::list<Point> &listOfPoints){
	std::list<Modele> listOfDroites;
	std::list<Point>  listWithoutPrecModelPoints = listOfPoints;
	Modele            m;
  bool              stopRansac = false;

	while (!stopRansac){
        //ransac(listOfPoints, n, NbPtPertinent, proba, seuil, NbPts)
        m = ransac(listWithoutPrecModelPoints, 2, 20, 0.99, 0.05, 20);
        if( std::abs(m.getCorrel()) > 0.5){
          maj(listWithoutPrecModelPoints, m);
          listOfDroites.push_back(m);
        }
        else {
          stopRansac = true;
        }
    }

    return listOfDroites;
}

Segment build(const std::list<Point> &points){
  Segment s;
  Point a;
  Point b;

  //on calcule les coordonnées des projetés orthogonaux des deux points extrêmes
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::min();

    for (auto &it : points){
      if (it.getX() < min){
        min = it.getX();
        a = it;
      }
      if (it.getX() > max){
        max = it.getX();
        b = it;
      }
    }

  //...puis la taille du segment en mètre
  double size = sqrt( (a.getX()-b.getX()) * (a.getX()-b.getX()) +
            (a.getY()-b.getY()) * (a.getY()-b.getY()));

  s.setPoints(a,b);
  s.setSize(size);

  return s;
}


std::list<Segment> buildSegment(Modele m, double seuil){
  std::list<Segment> listOfSegments;
  std::list<Point> tmp;
  std::list<Point>::const_iterator previousPoint = m.getPoints().cbegin();

  //pour chaque points dans la liste de points du modèle
  for(std::list<Point>::const_iterator it = m.getPoints().cbegin(); it != m.getPoints().cend(); ++it){
    //on calcule la distance entre voisins
    double d = sqrt((it->getY()-previousPoint->getY())*(it->getY()-previousPoint->getY()) +
                        (it->getX()-previousPoint->getX())*(it->getX()-previousPoint->getX()));
    
    //si les points sont proches
    if (d < seuil){
      //on sauvegarde ces points dans une liste
      tmp.push_back(*it);
    }
    //sinon (on détecte un seuil important)
    else {
      //on construit un segment à partir de la liste des points qui sont proches
      Segment s = build(tmp);

      //on recalcule l'angle à partir des points extrêmes
      double pente =  (s.getMax().getY()-s.getMin().getY()) /
              (s.getMax().getX()-s.getMin().getX());
      double angle = atan2(pente,1);
      s.setAngle(angle);

      //on enregistre le segment dans la liste de segments
      listOfSegments.push_back(s);
      tmp.clear();
      tmp.push_back(*it);
    }
    //on recommence en partant du point suivant le dernier point
    //qui était dans la liste précédente
    previousPoint = it;
  }
  //pour le dernier point, le seuil ne pouvant plus être dépassé,
  //on construit le dernier segment
  if (tmp.size() >= 2){
    Segment s = build(tmp);
    //on recalcule l'angle à partir des points extrêmes
    double pente =  (s.getMax().getY()-s.getMin().getY()) /
            (s.getMax().getX()-s.getMin().getX());
    double angle = atan2(pente,1);
    s.setAngle(angle);
    listOfSegments.push_back(s);
  }
  tmp.clear();

  return listOfSegments;
}

std::list<Segment> buildSegments(std::list<Modele> &listOfModeles){
  std::list<Segment> listOfSegments;
  //pour tous les modèles de la liste
  for (auto &it : listOfModeles){
    std::list<Segment> listTmp = buildSegment(it, 1);
    //on concatène les listes de segments trouvés à partir de chaque modèle ensemble
    listOfSegments.splice(listOfSegments.end(),listTmp);
  }

  return listOfSegments;
}

std::list<Machine> recognizeMachinesFrom(std::list<Segment> listOfSegments){
  std::list<Machine> tmp;
  int cpt = 1;

  for (auto &it : listOfSegments){
    double angle = it.getAngle();
    double size  = it.getSize();

    double  gM = 0.70 + 0.05,
        gm = 0.70 - 0.05,
        pM = 0.35 + 0.05,
        pm = 0.35 - 0.05;

    double  abscisse  = 0.0,
        ordonnee  = 0.0,
        orientation = 0.0;

    double absMilieu = (it.getMax().getX() + it.getMin().getX())/2;
    double ordMilieu = (it.getMax().getY() + it.getMin().getY())/2;

    //si c'est un grand côté
    if ((size > gm) && (size < gM)){
      abscisse = absMilieu + 0.35*sin(M_PI_2-angle);
      ordonnee = ordMilieu - 0.35*cos(M_PI_2-angle);
      orientation = angle;
    }
    //si c'est un petit côté
    if ((size > pm) && (size < pM)){
      abscisse = absMilieu + 0.175*sin(M_PI_2-angle);
      ordonnee = ordMilieu - 0.175*cos(M_PI_2-angle);
      orientation = M_PI_2 + angle;
    }

    if (abscisse != 0.0 && ordonnee != 0.0 && orientation != 0.0){
      Machine m(abscisse,ordonnee, orientation);
      tmp.push_back(m);
      m.clear();
    }
    cpt++;
  }

  return tmp;
}