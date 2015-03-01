#ifndef MODELE_H
#define MODELE_H

class Modele{
public:
	Modele();
	~Modele();

	Droite getDroite(){
		return m_droite;
	}
	double getCorrel(){
		return m_correl;
	}
	std::list<std::list<Point>::iterator> getIndex(){
		return m_index;
	}

	void addPoint(Point point){
		m_points.push_back(point);
	}
	void addIndex(std::list<Point>::iterator it){
		m_index.push_back(it);
	}
	void setDroite(Droite droite){
		m_droite=droite;
	}
 
	void linReg();
	void build(Point a, Point b);
	void constructFrom(Modele m);

private:
	Droite             					  m_droite;
	std::list<Point>   					  m_points;
	std::list<std::list<Point>::iterator> m_index;
	double             					  m_correl;
};

#endif