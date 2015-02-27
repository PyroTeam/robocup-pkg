#ifndef MODELE_H
#define MODELE_H

class Modele{
public:
	Modele();
	~Modele();

	Droite getDroite()    {return m_droite;}
	double getErreur()    {return m_erreur;}
	Point  getPoint(std::list<Point>::iterator it);

	void addPoint(Point point)    				{m_points.push_back(point);}
	void addIndex(std::list<Point>::iterator it){m_index.push_back(it);}
	void setDroite(Droite droite) 				{m_droite=droite;}
 
	void linReg();
	void build(Point a, Point b);

private:
	Droite             					  m_droite;
	std::list<Point>   					  m_points;
	std::list<std::list<Point>::iterator> m_index;
	double             					  m_erreur;
};

#endif