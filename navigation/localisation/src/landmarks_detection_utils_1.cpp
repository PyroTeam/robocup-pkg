#include "ros/ros.h"
#include "laserScan.h"
#include "Point.h"
#include "Droite.h"
#include "Modele.h"
#include "Segment.h"
#include "Machine.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"

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

Segment buildSegment(const std::list<Point> &points){
  Segment s;
  Point a;
  Point b;

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

  //...puis la taille du segment en mètre...
  double size = sqrt( (a.getX()-b.getX()) * (a.getX()-b.getX()) +
                      (a.getY()-b.getY()) * (a.getY()-b.getY()));

  //...et enfin l'angle
  double pente =  (b.getY()-a.getY()) / (b.getX()-a.getX());
  double angle = atan2(pente,1);
  
  s.setAngle(angle);
  s.setPoints(a,b);
  s.setSize(size);

  std::cout << "\n" << std::endl;
  std::cout << "Segment " << std::endl;
  std::cout << " Min(" << s.getMin().getX() << ", " << s.getMin().getY() << ")" << std::endl;
  std::cout << " Max(" << s.getMax().getX() << ", " << s.getMax().getY() << ")" << std::endl;
  std::cout << " taille : " << s.getSize() << std::endl;
  std::cout << " angle  : " << s.getAngle()*(180/M_PI) << std::endl;

  return s;
}

Machine calculateCoordMachine(Segment s){
  Machine m;

  double g = 0.70, p = 0.35, seuil = 0.05;
  double angle = s.getAngle();
  double size  = s.getSize();

  double absMilieu = (s.getMax().getX() + s.getMin().getX())/2;
  double ordMilieu = (s.getMax().getY() + s.getMin().getY())/2;

  geometry_msgs::Pose2D point;

  if ((size > p-seuil) && (size < p+seuil)){
      std::cout << "petit" << std::endl;
      point.x     = absMilieu + 0.175*sin(M_PI_2-angle);
      point.y     = ordMilieu - 0.175*cos(M_PI_2-angle);
      point.theta = atan2(tan(M_PI_2 + angle), 1);

      m.resetType();
  }
  else  if ((size > g-seuil) && (size < g+seuil)){
          std::cout << "grand" << std::endl;
          point.x     = absMilieu + 0.35*sin(M_PI_2-angle);
          point.y     = ordMilieu - 0.35*cos(M_PI_2-angle);
          point.theta = angle;

          m.setType();
        }

  m.setCentre(point);

  return m;
}

std::list<Machine> extractMachinesFrom(Modele m, double seuil){
  std::list<Machine> listOfMachines;
  std::list<Point> tmp;
  std::list<Point>::const_iterator previousPoint = m.getPoints().cbegin();

  double g = 0.70, p = 0.35;

  //pour chaque points dans la liste de points du modèle
  for(std::list<Point>::const_iterator it = m.getPoints().cbegin(); it != m.getPoints().cend(); ++it){
    Machine machine;
    //on calcule la distance entre voisins
    double d = sqrt((it->getY()-previousPoint->getY())*(it->getY()-previousPoint->getY()) +
                    (it->getX()-previousPoint->getX())*(it->getX()-previousPoint->getX()));

    //si les points sont proches
    if (d < seuil){
      //on sauvegarde ces points dans une liste
      tmp.push_back(*it);
    }
    //sinon si on a détecté un seuil
    else {
      Segment s = buildSegment(tmp);
      if(((s.getSize() > p-seuil) && (s.getSize() < p+seuil)) ||
         ((s.getSize() > g-seuil) && (s.getSize() < g+seuil))){
        //on construit une machine à partir du segment
        machine = calculateCoordMachine(s);
        //on enregistre la machine dans la liste de machines
        listOfMachines.push_back(machine);
        tmp.clear();
        tmp.push_back(*it);

        std::cout << "Machine ("<< machine.getCentre().x << ", " << machine.getCentre().y << ")" << std::endl;
        std::cout << "orientation : " << machine.getCentre().theta*(180/M_PI) << std::endl;
        std::cout << "\n" << std::endl;
      }
    }
    //on recommence en partant du point suivant le dernier point
    //qui était dans la liste précédente
    previousPoint = it;
  }
  tmp.clear();

  return listOfMachines;
}

std::list<Machine> convertModelesIntoMachines(std::list<Modele> &listOfModeles){
  std::list<Machine> listOfMachines;
  //pour tous les modèles de la liste
  for (auto &it : listOfModeles){
    std::list<Machine> listTmp = extractMachinesFrom(it,1);
    //on concatène les listes de segments trouvés à partir de chaque modèle ensemble
    listOfMachines.splice(listOfMachines.end(),listTmp);
  }

  return listOfMachines;
}