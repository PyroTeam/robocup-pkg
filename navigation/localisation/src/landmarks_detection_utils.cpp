#include "ros/ros.h"
#include "laserScan.h"
#include "Line.h"
#include "Model.h"
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

double dist(geometry_msgs::Point a, Line d)
{
    geometry_msgs::Point b;
    b.x = d.getPoint().x;
    b.y = d.getPoint().y;
    geometry_msgs::Point u;
    u.x = cos(d.getAngle());
    u.y = sin(d.getAngle());
    geometry_msgs::Point ba;
    ba.x = a.x-b.x;
    ba.y = a.y-b.y;

    return std::abs(u.x*ba.y - ba.x*u.y) / sqrt(u.x*u.x + u.y*u.y);
}

double dist(geometry_msgs::Point a, Segment s)
{
    geometry_msgs::Point b = s.getMin();
    geometry_msgs::Point u;
    u.x = cos(s.getAngle());
    u.y = sin(s.getAngle());
    geometry_msgs::Point ba;
    ba.x = a.x-b.x;
    ba.y = a.y-b.y;

    return std::abs(u.x*ba.y - ba.x*u.y) / sqrt(u.x*u.x + u.y*u.y);
}

geometry_msgs::Point ortho(geometry_msgs::Point a, Line d)
{
    double distance = dist(a,d);
    double dx = distance*cos(d.getAngle());
    double dy = distance*sin(d.getAngle());
    geometry_msgs::Point p;
    p.x = a.x - dx;
    p.y = a.y + dy;

    return p;
}

geometry_msgs::Point ortho(geometry_msgs::Point a, Segment s)
{
    double distance = dist(a,s);
    double dx = distance*cos(s.getAngle());
    double dy = distance*sin(s.getAngle());
    geometry_msgs::Point p;
    p.x = a.x - dx;
    p.y = a.y + dy;

    return p;
}

Model ransac(std::list<geometry_msgs::Point> &listOfPoints, int n, int NbPtPertinent, double proba, double seuil, int NbPts)
{
    int size = listOfPoints.size();
    int iter = 0, i = 0, j = 0;
    double w = double(NbPtPertinent)/double(size);
    double k = log10(1-proba)/log10(1-pow(w,n));

    Model meilleur_modele;

    while (iter < k)
    {
        Model modele_possible;
        geometry_msgs::Point a, b;
        //on prend deux points au hasard
        i = rand() % listOfPoints.size();
        do
        {
            j = rand() % listOfPoints.size();
        } while(j==i);

        //on stocke les indices de ces points dans la liste d'indices du modele possible
        //ainsi que les points correspondants
        int cpt = 0, NbPtFound = 0;
        for (std::list<geometry_msgs::Point>::iterator it = listOfPoints.begin(); it != listOfPoints.end(); ++it)
        {
            if (cpt == i)
            {
                a = *it;
                NbPtFound++;
            }
            if (cpt == j)
            {
                b = *it;
                NbPtFound++;
            }
            if (NbPtFound == 2)
            {
                break;
            }
            cpt++;
        }

        //on fabrique alors une droite à partir de ces points
        modele_possible.build(a,b);
        
        //pour tous les autres points
        for (std::list<geometry_msgs::Point>::iterator it = listOfPoints.begin(); it != listOfPoints.end(); ++it)
        {
            //si le point se situe dans le voisinage de la droite du modele_possible
            if (dist(*it, modele_possible.getLine()) < seuil)
            {
                //on ajoute l'indice correspondant à ce point dans la liste d'indices du modele_possible
                modele_possible.addIndex(it);
            }
        }

        //si le modele_possible contient assez de points
        if(modele_possible.getIndex().size() > NbPts)
        {
            //on fait une régression linéaire à partir des points appartenant au modèle
            //pour ajuster les paramètres de la droite et calculer le coeff de corrélation
            modele_possible.linReg();
        }

        //si le modele_possible est mieux que le meilleur_modele enregistré
        if (modele_possible.getIndex().size() > meilleur_modele.getIndex().size())
        {
        /*if (modele_possible.getCorrel() > meilleur_modele.getCorrel()){*/
            //on construit le nouveau meilleur_modele à partir du modele_possible
            meilleur_modele = modele_possible;
        }

        iter++;
    }
    meilleur_modele.linReg();
    meilleur_modele.update();

    return meilleur_modele;
}

void maj(std::list<geometry_msgs::Point> &list, Model m)
{
    const std::list<std::list<geometry_msgs::Point>::iterator> &indexes = m.getIndex();
    //pour tous les index contenus dans la liste d'index du modele
    for(std::list<std::list<geometry_msgs::Point>::iterator>::const_iterator it = indexes.cbegin(); it != indexes.cend(); ++it)
    {
        //on supprime dans la liste le point correspondant à l'index enregistré dans le meilleur_modele
        list.erase(*it);
    }
}

//rentrer tous les paramètres de RANSAC dans le prototype de findLines
std::list<Model> findLines(const std::list<geometry_msgs::Point> &listOfPoints, int NbPtPertinent, double seuil, int NbPts, std::list<geometry_msgs::Point> &l)
{
    std::list<Model>                 listOfDroites;
    std::list<geometry_msgs::Point>  listWithoutPrecModelPoints = listOfPoints;
    Model                            m;
    bool                             stopRansac = false;

    while (!stopRansac)
    {
        //  ransac(listOfPoints,               n, NbPtPertinent,proba, seuil, NbPts)
        m = ransac(listWithoutPrecModelPoints, 2, NbPtPertinent, 0.99, seuil, NbPts);

        if( std::abs(m.getCorrel()) > 0)
        {
            maj(listWithoutPrecModelPoints, m);
            listOfDroites.push_back(m);
        }
        else
        {
            stopRansac = true;
        }
    }

    l = listWithoutPrecModelPoints;

    return listOfDroites;
}

Segment build(const std::list<geometry_msgs::Point> &points)
{
    Segment s;
    geometry_msgs::Point a;
    a.x = 0;
    a.y = 1;
    geometry_msgs::Point b = a;

    //on calcule les coordonnées des projetés orthogonaux des deux points extrêmes
        double min =  std::numeric_limits<double>::max();
        double max = -std::numeric_limits<double>::max();

        for (auto &it : points)
        {
            if (it.x < min)
            {
                min = it.x;
                a = it;
            }
            if (it.x > max)
            {
                max = it.x;
                b = it;
            }
        }

    //...puis la taille du segment en mètre
    double size = sqrt( (a.x-b.x) * (a.x-b.x) +
                        (a.y-b.y) * (a.y-b.y));

    //...et enfin l'angle
    double pente =  (b.y-a.y) / (b.x-a.x);
    double angle = atan2(pente,1);
    
    s.setAngle(angle);
    s.setPoints(a,b);
    s.setSize(size);

    return s;
}


std::list<Segment> buildSegment(Model m, double seuil)
{
    std::list<Segment> listOfSegments;
    std::list<geometry_msgs::Point> tmp;
    std::list<geometry_msgs::Point>::const_iterator previousPoint = m.getPoints().cbegin();

    //pour chaque points dans la liste de points du modèle
    for(std::list<geometry_msgs::Point>::const_iterator it = m.getPoints().cbegin(); it != m.getPoints().cend(); ++it)
    {
        //on calcule la distance entre voisins
        double d = sqrt((it->y-previousPoint->y)*(it->y-previousPoint->y) +
                        (it->x-previousPoint->x)*(it->x-previousPoint->x));
        
        //si les points sont proches
        if (d < seuil)
        {
            //on sauvegarde ces points dans une liste
            tmp.push_back(*it);
        }
        //sinon (on détecte un seuil important)
        else
        {
            //on construit un segment à partir de la liste des points qui sont proches
            Segment s = build(tmp);

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
    if (tmp.size() >= 2)
    {
        Segment s = build(tmp);
        listOfSegments.push_back(s);
    }
    tmp.clear();

    return listOfSegments;
}

std::list<Segment> buildSegments(std::list<Model> &listOfModels)
{
    std::list<Segment> listOfSegments;
    //pour tous les modèles de la liste
    for (auto &it : listOfModels)
    {
        std::list<Segment> listTmp = buildSegment(it, 0.3);
        //on concatène les listes de segments trouvés à partir de chaque modèle ensemble
        listOfSegments.splice(listOfSegments.end(),listTmp);
    }

    return listOfSegments;
}

geometry_msgs::Pose2D& test(geometry_msgs::Pose2D &c1, geometry_msgs::Pose2D &c2)
{
    //test distance centre - points trouvés (distance Manhattan)
    float distC1 = std::abs(c1.x)*std::abs(c1.x) + std::abs(c1.y)*std::abs(c1.y);
    float distC2 = std::abs(c2.x)*std::abs(c2.x) + std::abs(c2.y)*std::abs(c2.y);
    if(distC1 < distC2)
    {
        return c2;
    }
    else
    {
        return c1;
    }
}

Machine calculateCoordMachine(Segment s)
{
    Machine m;

    double g = 0.70, p = 0.35, seuil = 0.05;
    double angle = s.getAngle();
    double size  = s.getSize();

    double absMilieu = (s.getMax().x + s.getMin().x)/2;
    double ordMilieu = (s.getMax().y + s.getMin().y)/2;

    geometry_msgs::Pose2D c1, c2, point;

    double tmp = atan2(tan(angle),1);

    if ((size > g-seuil) && (size < g+seuil))
    {
        c1.x = absMilieu - p/2*sin(angle);
        c1.y = ordMilieu + p/2*cos(angle);

        c2.x = absMilieu + p/2*sin(angle);
        c2.y = ordMilieu - p/2*cos(angle);

        m.setType(2);

        point = test(c1,c2);
        if (tmp < 0)
        {
            tmp += M_PI;
        }
        point.theta = tmp;
    }
    else
    {
        point.x     = 0.0;
        point.y     = 0.0;
        point.theta = 0.0;

        m.resetType();
    }

    m.setCentre(point);

    return m;
}

void maj(std::list<Segment> &list, Segment s){
    for(std::list<Segment>::iterator it = list.begin(); it != list.end(); ++it)
    {
        //on supprime dans la liste le point correspondant à l'index enregistré dans le meilleur_modele
        list.erase(it);
    }
}

std::vector<Machine> recognizeMachinesFrom(std::list<Segment> &listOfSegments)
{
    std::vector<Machine> tmp;

    for (auto &it : listOfSegments)
    {
        Machine m;
        m.setCentre(calculateCoordMachine(it).getCentre());

        //si on est en présence d'une machine
        if (m.getCentre().x != 0.0)
        {
            //si c'est la première détectée
            if (tmp.size() == 0)
            {
                tmp.push_back(m);
            }
            else
            {
                //sinon, pour chaque machine déjà stockée
                for (int i = 0; i < tmp.size(); ++i)
                {
                    //on calcule la distance entre le centre de la machine trouvée et la machine i
                    double d = sqrt((m.getCentre().x - tmp[i].getCentre().x)*(m.getCentre().x - tmp[i].getCentre().x) +
                                    (m.getCentre().y - tmp[i].getCentre().y)*(m.getCentre().y - tmp[i].getCentre().y));

                    //si la distance entre les centres trouvés permet de dire si on peut distinguer 2 machines
                    //rq : on met un seuil important puisque les zones sont de 1,5 * 2 m
                    if (d > 1)
                    {
                        tmp.push_back(m);
                    }
                    //sinon, on fait une moyenne des deux pour affiner la position du centre
                    else
                    {
                        geometry_msgs::Pose2D milieu;
                        milieu.x     = (m.getCentre().x     + tmp[i].getCentre().x)/2;
                        milieu.y     = (m.getCentre().y     + tmp[i].getCentre().y)/2;
                        milieu.theta = (m.getCentre().theta + tmp[i].getCentre().theta)/2;

                        //si la machine venait d'un petit côté
                        if (tmp[i].getType() == 1)
                        {
                            //on dit que la machine a été créée à partir d'un grand côté
                            tmp[i].setType(2);
                        }

                        //on met à jour la machine
                        tmp[i].setCentre(milieu);
                    }
                }
            }
        }
    }

    return tmp;
}

geometry_msgs::Pose2D pointToPose2D(geometry_msgs::Point point)
{
    geometry_msgs::Pose2D pose2d;
    pose2d.x = point.x;
    pose2d.y = point.y;
    pose2d.theta = 0.0;

    return pose2d;
}