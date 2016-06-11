#include "landmarks_detection_utils.h"
#include "math_functions.h"
#include "conversion_functions.h"
#include "Model.h"
#include "Machine.h"


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

        //pour tous les points
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
            //on construit le nouveau meilleur_modele à partir du modele_possible
            meilleur_modele = modele_possible;
        }

        iter++;
    }

    meilleur_modele.linReg();
    meilleur_modele.update();

    return meilleur_modele;
}

void reduceList(std::list<geometry_msgs::Point> &list, Model m)
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
        //  ransac(listOfPoints,               n, NbPtPertinent,proba, seuil, NbPts)
        m = ransac(listWithoutPrecModelPoints, 2, NbPtPertinent, 0.99, seuil, NbPts);

        if(m.getPoints().size() > NbPts)
        {
/*
            std::cout << "j'ai trouvé un modèle de " << m.getIndex().size() << " points" << std::endl;
            std::cout << "passant par le point (" << m.getLine().getPoint()->x << "," << m.getLine().getPoint()->y << ")" << std::endl;
            std::cout << "d'angle " << m.getLine().getAngle()*(180/M_PI) << std::endl;
            std::cout << "et dont la corrélation est de " << m.getCorrel() << std::endl;
*/
            reduceList(listWithoutPrecModelPoints, m);
            listOfDroites.push_back(m);
        }
        else
        {
            stopRansac = true;
        }
    }

    //l = listWithoutPrecModelPoints;

    return listOfDroites;
}

std::list<Segment> buildSegmentsFromOneModel(Model m, double seuil)
{
    Segment s;
    std::list<Segment> listOfSegments;
    std::list<geometry_msgs::Point> tmp;
    std::list<geometry_msgs::Point>::const_iterator previousPoint = m.getPoints().cbegin();

    //pour chaque points dans la liste de points du modèle
    for(std::list<geometry_msgs::Point>::const_iterator it = m.getPoints().cbegin(); it != m.getPoints().cend(); ++it)
    {
        //si les points sont proches
        if (geometry_utils::distance(*it, *previousPoint) < seuil)
        {
            //on sauvegarde ces points dans une liste
            tmp.push_back(*it);
        }
        //sinon (si on détecte un seuil important)
        else
        {
            if (geometry_utils::distance(tmp.front(), tmp.back()) >= 0.5)
            {
                //on construit un nouveau segment à partir de la liste enregistrée des points qui sont proches
                s.build(tmp);

                //on enregistre le segment dans la liste de segments
                listOfSegments.push_back(s);
            }
            //on clear la liste et on l'init avec le dernier point vu
            tmp.clear();
        }
        //on recommence en partant de ce point
        previousPoint = it;
    }

    //pour le dernier point, le seuil ne pouvant plus être dépassé,
    //on construit le dernier segment et on l'ajoute
    if (tmp.size() >= 2 && geometry_utils::distance(tmp.front(), tmp.back()) >= 0.5)
    {
        //on construit un nouveau segment à partir de la liste enregistrée des points qui sont proches
        s.build(tmp);

        //on enregistre le segment dans la liste de segments
        listOfSegments.push_back(s);
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
        if (std::abs(it.getSize() - 0.7) <= 0.05)
        {
            Machine m;
            m.calculateCoordMachine(it);
            tmp.push_back(m);
        }
    }

    return tmp;
}
