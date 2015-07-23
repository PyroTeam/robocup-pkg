#include "geometry_msgs/Point.h"
#include "Line.h"
#include "Model.h"
#include "Segment.h"
#include "landmarks_detection_utils.h"
#include "math_functions.h"

#include <ctime>
#include <cmath>
#include <limits>
#include <algorithm>

Model::Model() : m_correl(0.0)
{

}

Model::~Model()
{

}

Line Model::getLine() const 
{
	return m_line;
}

double Model::getCorrel() const
{
	return m_correl;
}

const std::list<std::list<geometry_msgs::Point>::iterator> &Model::getIndex() const
{
	return m_index;
}

const std::list<geometry_msgs::Point> &Model::getPoints() const
{
	return m_points;
}

void Model::addPoint(geometry_msgs::Point point)
{
	m_points.push_back(point);
}

void Model::setPoints(std::list<geometry_msgs::Point> listOfPoints)
{
	m_points = listOfPoints;
}

void Model::addIndex(std::list<geometry_msgs::Point>::iterator &it)
{
	m_index.push_back(it);
}

void Model::setLine(Line line)
{
	m_line = line;
}

void Model::linReg()
{
	std::list<geometry_msgs::Point> pts;
	geometry_msgs::Point p;
	for (auto &it : m_index)
	{
		p = *it;
		pts.push_back(p);
	}
	geometry_msgs::Pose2D pt;
	m_correl = ::linReg(pts, pt);
	m_line.set(pt);
}

void Model::build(geometry_msgs::Point a, geometry_msgs::Point b)
{
  	m_line.build(pointToPose2D(a),pointToPose2D(b));
}

void Model::update()
{
	m_points.clear();
	//pour tous les itérateurs contenu dans la liste d'index
	for (auto &it : m_index)
	{
		//it est une référence sur un élément de m_index
		//*it est le point recherché dans la liste de points
		m_points.push_back(*it);
	}
}

void Model::copy(Model m)
{
    setLine(m.getLine());
    m_correl = m.getCorrel();
    m_points = m.getPoints();
    m_index = m.getIndex();
}

void Model::ransac(std::list<geometry_msgs::Point> &listOfPoints, int n, int NbPtPertinent, double proba, double seuil, int NbPts)
{
    int size = listOfPoints.size();
    int iter = 0, i = 0, j = 0;
    double w = double(NbPtPertinent)/double(size);
    double k = log10(1-proba)/log10(1-pow(w,n));

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
        if (modele_possible.getIndex().size() > getIndex().size())
        {
            //on construit le nouveau meilleur_modele à partir du modele_possible
            copy(modele_possible);
        }

        iter++;
    }

    linReg();
    update();
}