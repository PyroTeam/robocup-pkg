#ifndef MODELE_H
#define MODELE_H

class Modele{
public:
	Modele();
	~Modele();

	Droite getDroite(){
		return m_droite;
	}
	Segment getSegment(){
		return m_segment;
	}
	double getCorrel(){
		return m_correl;
	}
	std::list<std::list<Point>::iterator> getIndex(){
		return m_index;
	}
	std::list<Point> getPoints(){
		return m_points;
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
	void buildSegment();

private:
	Droite             					  m_droite;
	Segment 							  m_segment;  //rempli uniquement si le modele est le meilleur_modele
	std::vector<Point>   				  m_points;   //rempli uniquement si le modele est le meilleur_modele
	std::list<std::list<Point>::iterator> m_index;
	double             					  m_correl;
};

#endif