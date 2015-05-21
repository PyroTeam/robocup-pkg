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

        if(m.getPoints().size() > NbPts) 
        {
/*
            std::cout << "j'ai trouvé un modèle de " << m.getIndex().size() << " points" << std::endl;
            std::cout << "passant par le point (" << m.getLine().getPoint().x << "," << m.getLine().getPoint().y << ")" << std::endl;
            std::cout << "d'angle " << m.getLine().getAngle()*(180/M_PI) << std::endl;
            std::cout << "et dont la corrélation est de " << m.getCorrel() << std::endl;
*/
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

double linReg(const std::list<geometry_msgs::Point> &points, geometry_msgs::Pose2D &p)
{

    int          n = points.size();
    double    sumX = 0.0, sumY = 0.0;
    double     ecX = 0.0,  ecY = 0.0;                   //ecart
    double sumEcXY = 0.0;                               //somme des produits des écarts sur x et y
    double    ec2X = 0.0, ec2Y = 0.0;                   //somme des écarts au carré
    double   covXY = 0.0, varX = 0.0, varY = 0.0;

    for(auto &it : points)
    {
        sumX  += it.x;
        sumY  += it.y;
    }

    //calcul des moyennes
    double moyX = sumX/double(n);
    double moyY = sumY/double(n);

    //calcul du coefficient de corrélation
    for(auto &it : points)
    {
        ecX   = it.x - moyX;
        ecY   = it.y - moyY;
        sumEcXY += ecX*ecY;

        ec2X += ecX*ecX;
        ec2Y += ecY*ecY;
    }

    covXY = sumEcXY/double(n);
    varX  = ec2X/double(n);
    varY  = ec2Y/double(n);

    double correl = covXY/sqrt(varX * varY);

    double slope     = covXY/varX;

    p.x = moyX;
    p.y = moyY;
    p.theta = atan2(slope,1);

    return correl*correl;
}

Segment build(const std::list<geometry_msgs::Point> &points){
    Segment s;
    geometry_msgs::Pose2D pose2d;

    //on fait une régression linéaire sur le segment
    double correl = linReg(points, pose2d);

    //on commence à remplir le segment
    s.setAngle(pose2d.theta);
    s.setPoints(pose2DToPoint(pose2d), pose2DToPoint(pose2d));
    //on projète alors les points extrèmes sur le segment linéarisé
    geometry_msgs::Point ptMin = ortho(points.front(),s);
    geometry_msgs::Point ptMax = ortho(points.back(),s);

    //et enfin on calcule la taille et l'angle du segment en mètre
    double size  = sqrt((ptMax.x-ptMin.x)*(ptMax.x-ptMin.x) + (ptMax.y-ptMin.y)*(ptMax.y-ptMin.y));
    double angle =  tan((ptMax.y-ptMin.y)/(ptMax.x-ptMin.x));

    s.setAngle(angle);
    s.setPoints(ptMin,ptMax);
    s.setSize(size);
/*
    std::cout << "Segment " << std::endl;
    std::cout << " Min(" << s.getMin().x << ", " << s.getMin().y << ")" << std::endl;
    std::cout << " Max(" << s.getMax().x << ", " << s.getMax().y << ")" << std::endl;
    std::cout << " taille : " << s.getSize() << std::endl;
    std::cout << " angle  : " << s.getAngle()*(180/M_PI) << std::endl;
    std::cout << " correlation  : " << correl << std::endl;
*/
    return s;
}

std::list<Segment> buildSegmentsFromOneModel(Model m, double seuil)
{
    std::list<Segment> listOfSegments;
    std::list<geometry_msgs::Point> tmp;
    std::list<geometry_msgs::Point>::const_iterator previousPoint = m.getPoints().cbegin();

    //pour chaque points dans la liste de points du modèle
    for(std::list<geometry_msgs::Point>::const_iterator it = m.getPoints().cbegin(); it != m.getPoints().cend(); ++it)
    {
        //on calcule la distance de proche en proche
        double d =  sqrt((it->y-previousPoint->y)*(it->y-previousPoint->y) +
                         (it->x-previousPoint->x)*(it->x-previousPoint->x));
        
        //si les points sont proches
        if (d < seuil)
        {
            //on sauvegarde ces points dans une liste
            tmp.push_back(*it);
        }
        //sinon (si on détecte un seuil important)
        else
        {
            //on construit un nouveau segment à partir de la liste enregistrée des points qui sont proches
            Segment s = build(tmp);

            if(s.getSize() > 0.35)
            {
                //on enregistre le segment dans la liste de segments
                listOfSegments.push_back(s);  
            }

            //on clear la liste et on l'init avec le dernier point vu
            tmp.clear();
            tmp.push_back(*it);
        }
        //on recommence en partant de ce point
        previousPoint = it;
    }

    //pour le dernier point, le seuil ne pouvant plus être dépassé,
    //on construit le dernier segment et on l'ajoute
    if (tmp.size() >= 20)
    {
        Segment s = build(tmp);
        if(s.getSize() > 0.35)
        {
            listOfSegments.push_back(s);
        }
    }
    tmp.clear();

    return listOfSegments;
}

std::list<Segment> buildSegmentsFromModels(std::list<Model> &listOfModels)
{
    std::list<Segment> listOfSegments;
    //pour tous les modèles de la liste
    for (auto &it : listOfModels)
    {
        std::list<Segment> listTmp = buildSegmentsFromOneModel(it, 0.2);
        //on concatène les listes de segments trouvés à partir de chaque modèle
        listOfSegments.splice(listOfSegments.end(),listTmp);
    }

    return listOfSegments;
}

Machine calculateCoordMachine(Segment s)
{
    Machine m;

    double angle = s.getAngle();
    double absMilieu = (s.getMax().x + s.getMin().x)/2;
    double ordMilieu = (s.getMax().y + s.getMin().y)/2;

    geometry_msgs::Pose2D center;

    //on met l'angle entre -M_PI_2 et M_PI_2
    angle = atan(tan(angle));
    //puis entre 0 et M_PI 
    //if (angle < 0)
    //{
    //    angle += M_PI;
    //}
    center.theta = angle;

    //si l'angle est > 0
    if (angle > 0.0)
    {
        center.x = absMilieu + 0.35/2*sin(angle);
        center.y = ordMilieu - 0.35/2*cos(angle);
    }
    //si l'angle est <= 0
    else 
    {
        center.x = absMilieu - 0.35/2*sin(angle);
        center.y = ordMilieu + 0.35/2*cos(angle);
    }

    m.setCentre(center);

    return m;
}

void maj(std::list<Segment> &list, Segment s)
{
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
        if (std::abs(it.getSize() - 0.7) <= 0.1)
        {
            tmp.push_back(calculateCoordMachine(it));
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

geometry_msgs::Point pose2DToPoint(geometry_msgs::Pose2D pose2d)
{
    geometry_msgs::Point point;
    point.x = pose2d.x;
    point.y = pose2d.y;

    return point;
}