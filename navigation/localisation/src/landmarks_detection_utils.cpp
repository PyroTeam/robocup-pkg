#include "landmarks_detection_utils.h"
#include "math_functions.h"
#include "conversion_functions.h"
#include "Model.h"
#include "Machine.h"

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
std::list<Model> findLines( const std::list<geometry_msgs::Point> &listOfPoints,
                            int NbPtPertinent,
                            double seuil,
                            int NbPts,
                            std::list<geometry_msgs::Point> &l)
{
    std::list<Model>                 listOfDroites;
    std::list<geometry_msgs::Point>  listWithoutPrecModelPoints = listOfPoints;
    Model                            m;
    bool                             stopRansac = false;

    while (!stopRansac)
    {
        //ransac(listOfPoints,               n, NbPtPertinent,proba, seuil, NbPts)
        m.ransac(listWithoutPrecModelPoints, 2, NbPtPertinent, 0.99, seuil, NbPts);

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
            Segment s;
            s.build(tmp);

            if(s.getSize() > 0.5)
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
    if (tmp.size() >= 2)
    {
        Segment s;
        s.build(tmp);
        if(s.getSize() > 0.5)
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

std::vector<Machine> recognizeMachinesFrom(std::list<Segment> &listOfSegments)
{
    std::vector<Machine> tmp;

    for (auto &it : listOfSegments)
    {
        if (std::abs(it.getSize() - 0.7) <= 0.1)
        {
            Machine m;
            m.calculateCoordMachine(it);
            tmp.push_back(m);
        }
    }

    return tmp;
}