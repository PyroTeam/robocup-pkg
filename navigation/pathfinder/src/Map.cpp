#include "pathfinder/Map.hpp"

	Map::Map()
	: _clean(true),
	_resolution(0.56),
	_width(19),
	_height(9),
	_origin_x(5.32),
	_origin_y(0.28)
	{		
		ROS_INFO("Objet Map, instanciation");	

		// Création des machines 
		_production_machine[0] = new Objet(MACHINE,(float)56/100,(float)168/100,-90);
		_production_machine[1] = new Objet(MACHINE,(float)56/100,(float)280/100,90);
		_production_machine[2] = new Objet(MACHINE,(float)168/100,(float)168/100,180);
		_production_machine[3] = new Objet(MACHINE,(float)168/100,(float)280/100,0);
		_production_machine[4] = new Objet(MACHINE,(float)168/100,(float)392/100,0);
		_production_machine[5] = new Objet(MACHINE,(float)280/100,(float)168/100,90);
		_production_machine[6] = new Objet(MACHINE,(float)280/100,(float)392/100,-90);
		_production_machine[7] = new Objet(MACHINE,(float)280/100,(float)504/100,180);
		_production_machine[8] = new Objet(MACHINE,(float)392/100,(float)168/100,0);
		_production_machine[9] = new Objet(MACHINE,(float)392/100,(float)280/100,90);
		_production_machine[10] = new Objet(MACHINE,(float)392/100,(float)504/100,-90);
		_production_machine[11] = new Objet(MACHINE,(float)504/100,(float)504/100,-90);
		_production_machine[12] = new Objet(MACHINE,(float)-56/100,(float)168/100,-90);
		_production_machine[13] = new Objet(MACHINE,(float)-56/100,(float)280/100,90);
		_production_machine[14] = new Objet(MACHINE,(float)-168/100,(float)168/100,180);
		_production_machine[15] = new Objet(MACHINE,(float)-168/100,(float)280/100,0);
		_production_machine[16] = new Objet(MACHINE,(float)-168/100,(float)392/100,0);
		_production_machine[17] = new Objet(MACHINE,(float)-280/100,(float)168/100,90);
		_production_machine[18] = new Objet(MACHINE,(float)-280/100,(float)392/100,-90);
		_production_machine[19] = new Objet(MACHINE,(float)-280/100,(float)504/100,180);
		_production_machine[20] = new Objet(MACHINE,(float)-392/100,(float)168/100,0);
		_production_machine[21] = new Objet(MACHINE,(float)-392/100,(float)280/100,90);
		_production_machine[22] = new Objet(MACHINE,(float)-392/100,(float)504/100,-90);
		_production_machine[23] = new Objet(MACHINE,(float)-504/100,(float)504/100,-90);

		_delivery_machine[0] = new Objet(DELIVERY,(float)534/100,(float)245/100,180);
		_delivery_machine[1] = new Objet(DELIVERY,(float)534/100,(float)280/100,180);
		_delivery_machine[2] = new Objet(DELIVERY,(float)534/100,(float)315/100,180);
		_delivery_machine[3] = new Objet(DELIVERY,(float)-534/100,(float)245/100,0);
		_delivery_machine[4] = new Objet(DELIVERY,(float)-534/100,(float)280/100,0);
		_delivery_machine[5] = new Objet(DELIVERY,(float)-534/100,(float)315/100,0);

		_recycling_machine[0] = new Objet(RECYCLING,(float)56/100,(float)504/100,0);
		_recycling_machine[1] = new Objet(RECYCLING,(float)-56/100,(float)504/100,180);

		// //Création des points
		// for (int i = 0; i < nbPointsLignes; ++i)
		// {
		// 	for (int j = 0; j < nbPointsColonnes; ++j)
		// 	{
		// 		_pointsPassage[i][j] = new Point(-5.04+j*0.56,0.56+i*0.56,i,j);
		// 	}
		// }				
		// _pointsPassage[2][2]->setType(INTERDIT);
		// _pointsPassage[2][4]->setType(INTERDIT);
		// _pointsPassage[2][6]->setType(INTERDIT);
		// _pointsPassage[2][8]->setType(INTERDIT);
		// _pointsPassage[2][10]->setType(INTERDIT);
		// _pointsPassage[2][12]->setType(INTERDIT);
		// _pointsPassage[2][14]->setType(INTERDIT);
		// _pointsPassage[2][16]->setType(INTERDIT);
		
		// _pointsPassage[4][2]->setType(INTERDIT);
		// _pointsPassage[4][6]->setType(INTERDIT);
		// _pointsPassage[4][8]->setType(INTERDIT);
		// _pointsPassage[4][10]->setType(INTERDIT);
		// _pointsPassage[4][12]->setType(INTERDIT);
		// _pointsPassage[4][16]->setType(INTERDIT);

		// _pointsPassage[6][4]->setType(INTERDIT);
		// _pointsPassage[6][6]->setType(INTERDIT);
		// _pointsPassage[6][12]->setType(INTERDIT);
		// _pointsPassage[6][14]->setType(INTERDIT);

		// _pointsPassage[8][0]->setType(INTERDIT);
		// _pointsPassage[8][2]->setType(INTERDIT);
		// _pointsPassage[8][4]->setType(INTERDIT);
		// _pointsPassage[8][8]->setType(INTERDIT);
		// _pointsPassage[8][10]->setType(INTERDIT);
		// _pointsPassage[8][14]->setType(INTERDIT);
		// _pointsPassage[8][16]->setType(INTERDIT);
		// _pointsPassage[8][18]->setType(INTERDIT);

		_allowDiagonal 		= true;
		_crossCorner 		= false;
		_heuristicFonction  = MANHATTAN;
		_poidsHeuristic 	= 1;

		ROS_INFO("Objet Map correctement instanciee");
	}



	void Map::gridCallback(nav_msgs::OccupancyGridConstPtr grid)
	{
	    ROS_INFO("Grid received");
	    constructMap(grid);
	}

	void Map::constructMap(nav_msgs::OccupancyGridConstPtr grid)
	{
		//Création des points
		for (int i = 0; i < _height; ++i)
		{
			for (int j = 0; j < _width; ++j)
			{
				_pointsPassage[i][j] = new Point(-_origin_x+_resolution/2+j*_resolution,_origin_y+_resolution/2+i*_resolution,i,j);
				if(grid->data[i*_width+j] == 100) {
					_pointsPassage[i][j]->setType(INTERDIT);
				} else {
					_pointsPassage[i][j]->setType(LIBRE);
				}
			}
		}		
	}

	void Map::constructMap()
	{
		//Création des points
		for (int i = 0; i < _height; ++i)
		{
			for (int j = 0; j < _width; ++j)
			{
				_pointsPassage[i][j] = new Point(-_origin_x+_resolution/2+j*_resolution,_origin_y+_resolution/2+i*_resolution,i,j);
			}
		}		
	}

	Map::~Map()
	{	
		// Desallocations des objets et points
		for (int i = 0; i < nbPointsLignes; ++i)
		{
			for (int j = 0; j < nbPointsColonnes; ++j)
			{
				delete _pointsPassage[i][j];
			}
		}			
		for (int i = 0; i < nbProductionMachine; ++i)
		{
			delete _production_machine[i];
		}
		for (int i = 0; i < nbDeliveryMachine; ++i)
		{
			delete _delivery_machine[i];
		}
		for (int i = 0; i < nbRecyclingMachine; ++i)
		{
			delete _recycling_machine[i];
		}
	}

	// AStar	
	void Map::setAllowDiagonal(bool allowDiagonal)
	{
		_allowDiagonal = allowDiagonal;
	}

	void Map::setCrossCorner(bool crossCorner)
	{
		_crossCorner = crossCorner;
	}

	void Map::setPoidsHeuristic(signed int poids)
	{
		_poidsHeuristic = poids;
	}

	// Utilise la fonction heuristic appropriee, selon notre choix de depart
	float Map::heuristic(Point const& pointDepart, Point const& pointDistant)
	{
		switch(_heuristicFonction)
		{
			case MANHATTAN:
				return heuristicManhattan(pointDepart,pointDistant);
			break;

			case EUCLIDEAN:
				return heuristicEuclidean(pointDepart,pointDistant);
			break;

			default:
				return -1;
			break;
		}
	}

	// Heuristic du taxi -> voir wikipedia, pertinent quand on ne se deplace pas ou peu en diagonale
	float Map::heuristicManhattan(Point const& pointDepart, Point const& pointDistant)
	{
		float dist  = 0;
		float distX = 0, distY = 0;

		distX = fabs(pointDistant.getX() - pointDepart.getX());
		distY = fabs(pointDistant.getY() - pointDepart.getY());

		dist  = distX + distY;

		return dist;
	}

	// Heuristic "vol d'oiseau"
	float Map::heuristicEuclidean(Point const& pointDepart, Point const& pointDistant)
	{
		float dist  = 0;
		float distX = 0, distY = 0;

		distX = fabs(pointDistant.getX() - pointDepart.getX());
		distY = fabs(pointDistant.getY() - pointDepart.getY());

		dist = std::sqrt(std::pow(distX,2)+std::pow(distY,2));

		return dist;
	}

	float Map::heuristicChebyshev(Point const& pointDepart, Point const& pointDistant)
	{
		float dist = 0;
		float distX = 0, distY = 0;

		distX = fabs(pointDistant.getX() - pointDepart.getX());
		distY = fabs(pointDistant.getY() - pointDepart.getY());

		dist = std::max(distX,distY);

		return dist;
	}

	void Map::setHeuristicFunction(typeHeuristic heuristicFonction)
	{
		_heuristicFonction = heuristicFonction;
	}

	bool Map::isFreeAt(signed int li, signed int col)
	{
		if(li < nbPointsLignes && li >= 0 && col < nbPointsColonnes && col>=0)
			return _pointsPassage[li][col]->isFree();
		else 
			return false;
	}

/*
 *
 * 	simpleOffsets:	diagonalOffsets:
 *  +---+---+---+    +---+---+---+
 *  |   | 0 |   |    | 0 |   | 1 |
 *  +---+---+---+    +---+---+---+
 *  | 3 |   | 1 |    |   |   |   |
 *  +---+---+---+    +---+---+---+
 *  |   | 2 |   |    | 3 |   | 2 |
 *  +---+---+---+    +---+---+---+
 *
 */
 /**
  * Retourne tous les voisins d'un point en prenant en compte les paramètres : 
  * _allowDiagonal et _crossCorner
  */
	signed int Map::getVoisins(std::vector<Point*> &voisins, Point *oirigin)
	{
		voisins.clear();
		signed int col, li;
		col = oirigin->getColonne();
		li  = oirigin->getLigne();

		bool s0 = false, d0 = false,
        	 s1 = false, d1 = false,
        	 s2 = false, d2 = false,
        	 s3 = false, d3 = false;


	    // ↑
	    if (isFreeAt(li - 1, col))
	    {
	        voisins.push_back(_pointsPassage[li - 1][col]);
	        s0 = true;
	    }
	    // →
	    if (isFreeAt(li, col + 1))
	    {
	        voisins.push_back(_pointsPassage[li][col + 1]);
	        s1 = true;
	    }
	    // ↓
	    if (isFreeAt(li + 1, col))
	    {
	        voisins.push_back(_pointsPassage[li + 1][col]);
	        s2 = true;
	    }
	    // ←
	    if (isFreeAt(li, col - 1))
	    {
	        voisins.push_back(_pointsPassage[li][col - 1]);
	        s3 = true;
	    }

	    if (!_allowDiagonal)
	        return 0;

	    // Les simple s0 à s3 remplient precedemment permettent 
	    // de trouver les diagonals d0 à d3 franchissable ou non
	    if (!_crossCorner)
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
	        voisins.push_back(_pointsPassage[li - 1][col - 1]);
	    }
	    // ↗
	    if (d1 && isFreeAt(li - 1, col + 1))
	    {
	        voisins.push_back(_pointsPassage[li - 1][col + 1]);
	    }
	    // ↘
	    if (d2 && isFreeAt(li + 1, col + 1))
	    {
	        voisins.push_back(_pointsPassage[li + 1][col + 1]);
	    }
	    // ↙
	    if (d3 && isFreeAt(li + 1, col - 1))
	    {
	        voisins.push_back(_pointsPassage[li + 1][col - 1]);
	    }
	    return 0;
	}

	// Algo AStar a proprement parler
	signed int Map::computeAStar(std::vector<Point*> &chemin,
								 Point *startPoint,
								 Point *endPoint)
	{
		std::cout << "computeAStar : start("
		<< startPoint->getLigne() << ','
		<< startPoint->getColonne() << ") | end("
		<< endPoint->getLigne() << ','
		<< endPoint->getColonne() << ")" << std::endl;

		// Il faut remettre a zero tout AStar
		chemin.clear();
		reset();
		std::multiset<Point*,CompareF> aEvaluer;	// Tableau associatifs de points tries selon F
		std::multiset<Point*> dejaEvalue;
		std::vector<Point*> voisins;
		unsigned int i;

		aEvaluer.insert(startPoint);				// On commence l'evaluation par le point de depart

		while(!aEvaluer.empty())					// Tant qu'il reste des points a evaluer, on persevere
		{
			Point *p;

			p = *(aEvaluer.begin());				// On commence par evaluer le point avec le plus petit F (au tout debut il n'y a que start)

			//ROS_INFO("\nA Evaluer Contains :");
			//Affichage du tableau de points à évaluer

 			/*for (std::multiset<Point*>::iterator it=aEvaluer.begin();
 				 it!=aEvaluer.end();
 				 ++it )
 			{
 				std::cout << " ("
 				<< (*it)->getLigne() << ','
 				<< (*it)->getColonne() << "):"
 				<< (*it)->getF();
 			} 
 			std::cout << '\n';
 
 			//Affichage du point évalué

         	std::cout << "Point evalue: ("
 				<< p->getLigne() << ','
 				<< p->getColonne() << "):"
				<< " f :" << p->getF()
				<< " g :" << p->getG()
				<< " h :" << p->getH()
				<< std::endl;*/

			// Si le point a evaluer est le point d'arrive, on a trouve notre chemin
			if(p == endPoint)
			{
				ROS_INFO("Arrive au point terminal !");

				// Il faut reconstruire le chemin en remontant de parents en parents
				Point* prec;
                int count = 0;
				do
				{
					count++;
		            ROS_INFO("Count = %d", count);

					chemin.push_back(p);
					p->getPointPrec(prec);
					p = prec;

				} while(prec != NULL);

				// Le chemin est constuit a l'envers, il faut le retourner
				std::reverse(chemin.begin(),chemin.end());

                setClean(false);
				return 0;
			}

			// Le point que l'on evalue passe dans la liste deja evaluees
			dejaEvalue.insert(p);
			aEvaluer.erase(p);
			getVoisins(voisins,p);

			// On va ajouter tous les voisins a la liste aEvaluer, en construisant leurs infos F, G et H
			// Ils seront automatiquement tries par F croissant, ainsi au prochain tours on evaluera celui le plus proche de l'arrivee
			// (celui avec le F le plus petit)
			//ROS_INFO("Non evalue, evaluation des voisins");
			for (i = 0; i < voisins.size(); i++)
			{
				// Si voisins[i] a déjà été évalué - iteration suivante
				if(dejaEvalue.count(voisins[i]) > 0 && voisins[i])
				{
					continue;
				}

				// Sinon - calcul g potentiel
				signed int newG = p->getG() + voisins[i]->distWith(*p);

				// Si le voisins[i] n'et pas deja dans a evalue 
				// ou que le nouveau g est plus interessant
				// on modifie et on stocke

				if( aEvaluer.count(voisins[i]) == 0 ||
					newG < voisins[i]->getG())
				{
					voisins[i]->setPointPrec(p);
					voisins[i]->setG(newG);
					voisins[i]->setH((voisins[i]->getH())?voisins[i]->getH():heuristic(*voisins[i], *endPoint));
					voisins[i]->setF(voisins[i]->getG()+_poidsHeuristic*voisins[i]->getH());
					
					aEvaluer.insert(voisins[i]);
				}
			}
		}

		setClean(false);
		return 0;		
	}


	signed int Map::getPointAt(signed int ligne, signed int colonne, Point*& point) const
	{
		if( ligne < nbPointsLignes &&
			ligne >= 0 &&
			colonne < nbPointsColonnes &&
			colonne>=0)
		{
			point = _pointsPassage[ligne][colonne];
			return 0;
		}
		else 
		{
			return -1;
		}
	}


	signed int Map::getNearestPoint(float x, float y, Point*& point) const
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
				if(_pointsPassage[l][c]->isFree())
				{
					dx = x - _pointsPassage[l][c]->getX();
					dy = y - _pointsPassage[l][c]->getY();
					dist = dx*dx+dy*dy;

					if(dist < minDist)
					{
						minDist = dist;
						point = _pointsPassage[l][c];
					}
				}
			}
		}

		return 0;
	}


	void Map::reset()
	{
		if(!getClean())
		{
			for (int i = 0; i < nbPointsLignes; ++i)
			{
				for (int j = 0; j < nbPointsColonnes; ++j)
				{
					_pointsPassage[i][j]->reset();
				}
			}		

			setClean(true);	
		}
	}

	bool Map::getClean()
	{
		return _clean;
	}

	void Map::setClean(bool c)
	{
		_clean = c;
	}