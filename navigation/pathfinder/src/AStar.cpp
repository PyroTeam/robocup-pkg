/**
 * \file AStar.cpp
 * \brief Implémentation de l'algorithme AStar.
 * \author Valentin Vergez
 * \version 1.0.1
 * \date 16 mai 2015
 *
 * Implémentation complète de l'algorithme AStar pour les robotinos de PYRO Team,
 * dans le cadre de la RoboCup Logistic League.
 *
 * La classe AStar est en charge de récupérer la map (une OccupancyGrid), grâce
 * à sa fonction de CallBack et de trouver un chemin optimal entre deux points.
 * Fonction : computeAStar.
 *
 * L'implémentation de l'openList est en grande partie codée dans Point.cpp
 *
 */

/*==========  Includes  ==========*/
#include "pathfinder/AStar.hpp"
#include "common_utils/Chrono.h"

/*==========  Constructor & desctuctors  ==========*/
/**
 * @brief Constructeur par défaut
 * @details L'allocation mémoire de la map est faite ici.
 * Certains paramètres de l'algo sont aussi rensignés ici.
 */
AStar::AStar()
: m_clean(true),
m_resolution(0.05),
m_width(280),
m_height(180),
m_origin_x(-7),
m_origin_y(-2)
{
    m_allowDiagonal      = true;
    m_crossCorner        = false;
    m_heuristicFonction  = EUCLIDEAN;
    m_poidsHeuristic     = 1;

    constructMap();

    ROS_INFO("Objet AStar correctement instanciee (EUCLIDEAN)");
}

/**
 * @brief Destructeur par défaut
 * @details Desalloue la mémoire de la map
 */
AStar::~AStar()
{
    destructMap();
}

/*==========  Fonctions relatives à la map/grid  ==========*/
/**
 * @brief Callback nav_msgs::OccupancyGrid
 * @details Met simplement à jour la map à chaque réception de l'OccupancyGrid
 *
 * @param grid OccupancyGrid représentant la map et les obstacles
 */
void AStar::gridCallback(nav_msgs::OccupancyGridConstPtr grid)
{
    updateMap(grid);
}

/**
 * @brief Fonction de mise à jour de la map
 * @details Met à jour les points selon les informations de l'OccupancyGrid
 * Les emplacements mémoires des points de la map sont inaltérés ici
 *
 * @param grid OccupancyGrid représentant la map et les obstacles
 */
void AStar::updateMap(nav_msgs::OccupancyGridConstPtr grid)
{
    //Mise à jour des points
    for (int i = 0; i < m_height; ++i)
    {
        for (int j = 0; j < m_width; ++j)
        {
            if(grid->data[i*m_width+j] != 0) {
                m_pointsPassage[i][j]->setType(INTERDIT);
            } else {
                m_pointsPassage[i][j]->setType(LIBRE);
            }
        }
    }
}

/**
 * @brief Création de la map
 * @details La map est crée dans cette fonction.
 * i.e : chaque point de la map sont alloués en mémoires et placées à leur coordonées
 *
 * A ce stade la map est vierge, pas de murs, pas d'obstacles
 */
void AStar::constructMap()
{
    //Création des points en mémoire
    for (int i = 0; i < m_height; ++i)
    {
        for (int j = 0; j < m_width; ++j)
        {
            delete m_pointsPassage[i][j];
            m_pointsPassage[i][j] = new Point(m_origin_x+m_resolution/2+j*m_resolution,m_origin_y+m_resolution/2+i*m_resolution,i,j);
        }
    }
}

/**
 * @brief Destruction de la map
 * @details La mémoire allouée pour les points de la map est libérée
 */
void AStar::destructMap()
{
    //Création des points
    for (int i = 0; i < m_height; ++i)
    {
        for (int j = 0; j < m_width; ++j)
        {
            delete m_pointsPassage[i][j];
        }
    }
}

/*==========  Fonctions relatives à l'algo AStar  ==========*/
void AStar::setAllowDiagonal(bool allowDiagonal)
{
    m_allowDiagonal = allowDiagonal;
}

void AStar::setCrossCorner(bool crossCorner)
{
    m_crossCorner = crossCorner;
}

void AStar::setPoidsHeuristic(signed int poids)
{
    m_poidsHeuristic = poids;
}

void AStar::setHeuristicFunction(typeHeuristic heuristicFonction)
{
    m_heuristicFonction = heuristicFonction;
}

/**
 * @brief Fonction d'estimation de distance heuristique entre deux points
 * @details Utilise la fonction heuristique appropriee, selon notre configuration de depart
 *
 * @param pointDepart
 * @param pointDistant
 *
 * @return La distance estimee entre les deux points
 */
float AStar::heuristic(Point const *pointDepart, Point const *pointDistant)
{
    switch(m_heuristicFonction)
    {
        case MANHATTAN:
            return heuristicManhattan(pointDepart,pointDistant);
        break;

        case EUCLIDEAN:
            return heuristicEuclidean(pointDepart,pointDistant);
        break;

        case CHEBYSHEV:
            return heuristicChebyshev(pointDepart,pointDistant);
        break;

        default:
            ROS_ERROR("AStar::heuristic : Heuristique %d INCONNU", m_heuristicFonction);
            return -1;
        break;
    }
}

/**
 * @brief Distance heuristique de Manhattan
 * @details Heuristic du taxi -> voir wikipedia, pertinent quand on ne se deplace pas ou peu en diagonale
 *
 * @param pointDepart
 * @param pointDistant
 *
 * @return La distance estimee entre les deux points
 */
float AStar::heuristicManhattan(Point const *pointDepart, Point const *pointDistant)
{
    float dist  = 0;
    float distX = 0, distY = 0;

    distX = fabs(pointDistant->getX() - pointDepart->getX());
    distY = fabs(pointDistant->getY() - pointDepart->getY());

    dist  = distX + distY;

    return dist;
}

/**
 * @brief Distance heuristique d'Euclide
 * @details Heuristic "vol d'oiseau"
 *
 * @param pointDepart
 * @param pointDistant
 *
 * @return La distance estimee entre les deux points
 */
float AStar::heuristicEuclidean(Point const *pointDepart, Point const *pointDistant)
{
    float dist  = 0;
    float distX = 0, distY = 0;

    distX = fabs(pointDistant->getX() - pointDepart->getX());
    distY = fabs(pointDistant->getY() - pointDepart->getY());

    dist = std::sqrt(distX*distX + distY*distY);

    return dist;
}

/**
 * @brief Distance heuristique de Chebyshev
 * @details Retourne la distance en X ou en Y maximale
 *
 * @param pointDepart
 * @param pointDistant
 *
 * @return La distance estimee entre les deux points
 */
float AStar::heuristicChebyshev(Point const *pointDepart, Point const *pointDistant)
{
    float dist = 0;
    float distX = 0, distY = 0;

    distX = fabs(pointDistant->getX() - pointDepart->getX());
    distY = fabs(pointDistant->getY() - pointDepart->getY());

    dist = std::max(distX,distY);

    return dist;
}

/**
 * @brief Indique si un point est libre (passage possible) à la ligne/colonne indiquée
 * @details Si un point est configuré comme non libre (obstacle ou autre)
 * ou est exterieur à la map, le point est condiséré impossible à franchir
 *
 * @param raw
 * @param col
 *
 * @return Booleen, vrai pour libre, faux sinon
 */
bool AStar::isFreeAt(signed int raw, signed int col)
{
    if(raw < nbPointsLignes && raw >= 0 && col < nbPointsColonnes && col>=0)
        return m_pointsPassage[raw][col]->isFree();
    else
        return false;
}

/**
 * @brief Retourne tous les voisins d'un point
 * @details
 * Retourne tous les voisins d'un point en prenant en compte les paramètres :
 * m_allowDiagonal et m_crossCorner
 *
 * @param voisins Vecteur parametre resultat
 * @param oirigin Point dont on cherche les voisins
 *
 *  s0 à 4 représentent les voisins simples
 *  d0 à 4 représentent les voisins diagonaux
 *
 *  simpleOffsets:  diagonalOffsets:
 *  +---+---+---+    +---+---+---+
 *  |   | 0 |   |    | 0 |   | 1 |
 *  +---+---+---+    +---+---+---+
 *  | 3 |   | 1 |    |   |   |   |
 *  +---+---+---+    +---+---+---+
 *  |   | 2 |   |    | 3 |   | 2 |
 *  +---+---+---+    +---+---+---+
 *
 * @return 0 si tous s'est bien passé
 */
signed int AStar::getVoisins(std::vector<Point*> &voisins, Point *oirigin)
{
    voisins.clear();
    signed int col, li;
    col = oirigin->getColumn();
    li  = oirigin->getRaw();

    bool s0 = false, d0 = false,
         s1 = false, d1 = false,
         s2 = false, d2 = false,
         s3 = false, d3 = false;


    // ↑
    if (isFreeAt(li - 1, col))
    {
        voisins.push_back(m_pointsPassage[li - 1][col]);
        s0 = true;
    }
    // →
    if (isFreeAt(li, col + 1))
    {
        voisins.push_back(m_pointsPassage[li][col + 1]);
        s1 = true;
    }
    // ↓
    if (isFreeAt(li + 1, col))
    {
        voisins.push_back(m_pointsPassage[li + 1][col]);
        s2 = true;
    }
    // ←
    if (isFreeAt(li, col - 1))
    {
        voisins.push_back(m_pointsPassage[li][col - 1]);
        s3 = true;
    }

    if (!m_allowDiagonal)
        return 0;

    // Les simple s0 à s3 remplient precedemment permettent
    // de trouver les diagonals d0 à d3 franchissable ou non
    if (!m_crossCorner)
    {
        d0 = s3 && s0;
        d1 = s0 && s1;
        d2 = s1 && s2;
        d3 = s2 && s3;
    }
    else
    {
        d0 = s3 || s0;
        d1 = s0 || s1;
        d2 = s1 || s2;
        d3 = s2 || s3;
    }

    // ↖
    if (d0 && isFreeAt(li - 1, col - 1))
    {
        voisins.push_back(m_pointsPassage[li - 1][col - 1]);
    }
    // ↗
    if (d1 && isFreeAt(li - 1, col + 1))
    {
        voisins.push_back(m_pointsPassage[li - 1][col + 1]);
    }
    // ↘
    if (d2 && isFreeAt(li + 1, col + 1))
    {
        voisins.push_back(m_pointsPassage[li + 1][col + 1]);
    }
    // ↙
    if (d3 && isFreeAt(li + 1, col - 1))
    {
        voisins.push_back(m_pointsPassage[li + 1][col - 1]);
    }
    return 0;
}

/**
 * @brief Implementation d'AStar a proprement parler
 * @details [long description]
 *
 * @param path Vecteur résultat contenant les points du chemin
 * @param startPoint
 * @param endPoint
 * @return 0 si tous s'est bien passé, -1 sinon
 */
signed int AStar::computeAStar(std::vector<Point*> &path,
                             Point *startPoint,
                             Point *endPoint)
{
    common_utils::HighResChrono chrono;
    chrono.start();

// Environement
    // Il faut remettre a zero tout AStar
    reset();
    Point* openList = NULL;

// Algo
    // Le point de depart sera le premier evalue
    startPoint->insertInOpenList(&openList, openList);

    // Tant qu'il reste des points a evaluer, on persevere
    while(!Point::openListIsEmpty(openList))
    {
        ROS_DEBUG_STREAM("OpenList size : " << Point::openListSize());
        ROS_DEBUG_STREAM("CloseList size : " << Point::closeListSize());
        // if(Point::openListSize() != Point::countList(openList))
        // {
        //     ROS_ERROR("Incoherence size et count");
        //     exit(666);
        // }

        // A chaque iteartion, on evalue le point avec le plus petit F
        //  (au tout debut il n'y a que startPoint)
        Point *actualPoint = Point::getLowerFromOpenList(openList);

        if(!actualPoint->isInOpenList())
        {
            ROS_ERROR("ActualPoint not int OpenList");

            ROS_WARN("ActualPoint %d;%d (%f;%f) %f|%f|%f"
                , actualPoint->getRaw()
                , actualPoint->getColumn()
                , actualPoint->getX()
                , actualPoint->getY()
                , actualPoint->getF()
                , actualPoint->getG()
                , actualPoint->getH());

            exit(666);
        }

        // On passe le point d'open a close list
        actualPoint->removeFromOpenList(&openList);
        actualPoint->insertInCloseList();

        // Si le point a evaluer n'est pas le point d'arrive, on continue AStar
        if(actualPoint != endPoint)
        {
            // On recupere tous ses voisins
            std::vector<Point*> neighbours;
            getVoisins(neighbours,actualPoint);
            unsigned int newPointCounter = 0;
            unsigned int updatedPointCounter = 0;
            static unsigned int s_totalUpdatedPointCounter = 0;

            // On va ajouter tous les voisins a la liste openList, en construisant leurs infos F, G et H
            // Ils seront automatiquement tries par F croissant, ainsi au prochain tours on evaluera celui le plus proche de l'arrivee
            // (celui avec le F le plus petit)
            for (auto &neighbour : neighbours)
            {
                // Si neighbour a déjà été évalué - iteration suivante
                if(neighbour->isInCloseList())
                {
                    continue;
                }

                // Sinon - calcul g potentiel via actualPoint
                float newG = actualPoint->getG() + neighbour->distWith(actualPoint);

                // Si le neighbour n'et pas deja dans a evalue
                // ou que le nouveau g est plus interessant
                // on modifie et on stocke
                if( !neighbour->isInOpenList() ||
                    newG < neighbour->getG())
                {
                    if(!neighbour->isInOpenList())
                    {
                        neighbour->setPointPrec(actualPoint);
                        neighbour->setG(newG);
                        neighbour->setH((neighbour->getH())?neighbour->getH():heuristic(neighbour, endPoint));
                        neighbour->setF(neighbour->getG()+m_poidsHeuristic*neighbour->getH());

                        neighbour->insertInOpenList(&openList);
                        newPointCounter++;


                        // ROS_WARN("Ajout %d;%d "
                        //     , neighbour->getRaw()
                        //     , neighbour->getColumn());
                    }
                    else
                    {
                        neighbour->removeFromOpenList(&openList);

                        float oldF = neighbour->getF();

                        neighbour->setPointPrec(actualPoint);
                        neighbour->setG(newG);
                        neighbour->setH((neighbour->getH())?neighbour->getH():heuristic(neighbour, endPoint));
                        neighbour->setF(neighbour->getG()+m_poidsHeuristic*neighbour->getH());

                        neighbour->insertInOpenList(&openList);

                        updatedPointCounter++;
                        s_totalUpdatedPointCounter++;

                        // ROS_WARN("Update %d;%d NEWF %f OLD %f"
                        //     , neighbour->getRaw()
                        //     , neighbour->getColumn()
                        //     , neighbour->getF()
                        //     , oldF);
                    }
                }
            }
            ROS_DEBUG("Ajout de %d point(s), MAJ de %d point(s)",newPointCounter,updatedPointCounter);
            ROS_DEBUG("MAJ TOTAL de %d point(s)",s_totalUpdatedPointCounter);

        }
        // Sinon si le point a evaluer est le point d'arrive, on a trouve notre chemin
        else
        {
            ROS_INFO("PathFinder termine");

            // Il faut reconstruire le chemin en remontant de parents en parents
            path.clear();
            Point* previousPoint = NULL;
            do
            {
                path.push_back(actualPoint);
                actualPoint->getPointPrec(previousPoint);
                actualPoint = previousPoint;

            } while(previousPoint != NULL);

            ROS_DEBUG("NB points chemin : %lu", path.size());

            // Le chemin est constuit a l'envers, il faut le retourner
            std::reverse(path.begin(),path.end());

            setClean(false);

            chrono.stop();
            ROS_INFO_STREAM("Time to find the path : " << chrono);

            return 0;
        }
    }

    ROS_WARN("PathFinder termine sans trouver de chemin");

    setClean(false);
    return -1;
}

/**
 * @brief Retourne un point de la map repéré par sa ligne et sa colonne
 *
 * @param ligne
 * @param colonne
 * @param point Paramètre résultat
 * @return 0 si le point existe, -1 sinon
 */
signed int AStar::getPointAt(signed int ligne, signed int colonne, Point*& point) const
{
    if( ligne < nbPointsLignes &&
        ligne >= 0 &&
        colonne < nbPointsColonnes &&
        colonne>=0)
    {
        point = m_pointsPassage[ligne][colonne];
        return 0;
    }
    else
    {
        return -1;
    }
}

/**
 * @brief Retourne le point de la map le plus proche
 *
 * @param x
 * @param y
 * @param point Paramètre résultat
 * @return 0 si tous s'est bien passé, -1 sinon
 */
signed int AStar::getNearestPoint(float x, float y, Point*& point) const
{
    if(nbPointsLignes == 0 || nbPointsColonnes == 0)
    {
        return -1;
    }

    int l, c;
    float dist, dx, dy, minDist = FLT_MAX;

    for(l=0; l<nbPointsLignes; l++)
    {
        for(c=0; c<nbPointsColonnes; c++)
        {
            if(m_pointsPassage[l][c]->isFree())
            {
                dx = x - m_pointsPassage[l][c]->getX();
                dy = y - m_pointsPassage[l][c]->getY();
                dist = dx*dx+dy*dy;

                // ROS_INFO("%d:%d  %f:%f dist %f",l,c,_pointsPassage[l][c]->getX(),_pointsPassage[l][c]->getY(),dist);

                if(dist < minDist)
                {
                    minDist = dist;
                    point = m_pointsPassage[l][c];
                }
            }
        }
    }

    return 0;
}

/**
 * @brief Remise à zero d'AStar
 * @details Ne remet a zero AStar que si necessaire,
 * grâce au booleen m_clean qui indique si un reset d'AStar serait superflu
 */
void AStar::reset()
{
    if(!getClean())
    {
        for (int i = 0; i < nbPointsLignes; ++i)
        {
            for (int j = 0; j < nbPointsColonnes; ++j)
            {
                m_pointsPassage[i][j]->reset();
            }
        }

        setClean(true);
    }
}

void AStar::setClean(bool c)
{
    m_clean = c;
}

bool AStar::getClean()
{
    return m_clean;
}
