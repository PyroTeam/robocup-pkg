#ifndef MODELE_H
#define MODELE_H

class Modele{
public:
	Modele();
	~Modele();

	Droite getDroite()    {return m_droite;}
	Point getPoint(int i) {return m_points[i];}
	double getErreur()    {return m_erreur;}

	void addPoint(Point point)    {m_points.push_back(point);}
	void setDroite(Droite droite) {m_droite=droite;}

	Modele initModele();
	void linReg();
private:
	Droite             m_droite;
	std::vector<Point> m_points;
	double             m_erreur;
};

#endif