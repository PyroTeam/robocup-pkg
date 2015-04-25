#include "pathfinder/Point.hpp"


	Point::Point(float x, float y, typePoint type)
	{
		setPosition(x,y);
		setType(type);

		setH(0);
		setG(0);
		setF(0);
		setPointPrec(NULL);
		setLigne(-1);
		setColonne(-1);
	}

	Point::Point(float x, float y, signed int ligne, signed int colonne, typePoint type)
	{
		setPosition(x,y);
		setType(type);


		setH(0);
		setG(0);
		setF(0);
		setPointPrec(NULL);
		setLigne(ligne);
		setColonne(colonne);
	}

	Point::~Point()
	{
	}

	int Point::setPosition(float x, float y)
	{
		_x = x;
		_y = y;

		return 0;
	}

	int Point::setType(typePoint type)
	{
		_type = type;

		return 0;
	}

	float Point::getX() const
	{
		return _x;
	}

	float Point::getY() const
	{
		return _y;
	}

	typePoint Point::getType() const
	{
		return _type;
	}

	// A Star
	int Point::setPointPrec(Point *pointPrecedent)
	{
		_pointPrecedent = pointPrecedent;
		return 0;
	}

	int Point::getPointPrec(Point* &pointPrecedent)
	{
		pointPrecedent = _pointPrecedent;
		return 0;
	}

	float Point::getH() const
	{
		return _h;
	}

	float Point::getG() const
	{
		return _g;
	}

	float Point::getF() const
	{
		return _f;
	}

	void Point::setH(float h)
	{
		_h = h;
	}

	void Point::setG(float g)
	{
 		_g = g;
	}

	void Point::setF(float f)
	{
		_f = f;
	}

	signed int Point::getLigne() const
	{
		return _ligne;
	}

	signed int Point::getColonne() const
	{
		return _colonne;
	}

	void Point::setLigne(signed int ligne)
	{
		_ligne = ligne;
	}

	void Point::setColonne(signed int colonne)
	{
		_colonne = colonne;
	}

	bool Point::isFree()
	{
		return (_type == LIBRE);
	}

	float Point::distWith(Point const& pointDistant) const
	{
		float dist  = 0;
		float distX = 0, distY = 0;

		distX = fabs(pointDistant.getX() - getX());
		distY = fabs(pointDistant.getY() - getY());

		dist = std::sqrt(std::pow(distX,2)+std::pow(distY,2));

		return dist;
	}

	void Point::reset()
	{
		setH(0);
		setG(0);
		setF(0);
		setPointPrec(NULL);
	}


	bool operator<(Point const& p1, Point const& p2)
	{
		return p1.getF()<p2.getF();
	}
	bool operator<=(Point const& p1, Point const& p2)
	{
		return p1.getF()<=p2.getF();
	}
	bool operator>(Point const& p1, Point const& p2)
	{
		return p1.getF()>p2.getF();
	}
	bool operator>=(Point const& p1, Point const& p2)
	{
		return p1.getF()>=p2.getF();
	}
	bool operator==(Point const& p1, Point const& p2)
	{
		return (p1.getLigne() == p2.getLigne() && p1.getColonne() == p2.getColonne());
	}
