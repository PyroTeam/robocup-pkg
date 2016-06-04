#ifndef MODEL_H
#define MODEL_H

#include "geometry_msgs/Point.h"
#include "conversion_functions.h"
#include "Line.h"

#include <list>

class Model{
public:
	Model();
	~Model();

	Line getLine() const;
	double getCorrel() const;
	const std::list<std::list<geometry_msgs::Point>::iterator> &getIndex() const;
	const std::list<geometry_msgs::Point> &getPoints() const;

	void addPoint(const geometry_msgs::Point &point);
	void setPoints(std::list<geometry_msgs::Point> listOfPoints);
	void addIndex(std::list<geometry_msgs::Point>::iterator &it);
	void setLine(const Line &line);

	void linReg();
	void build(const geometry_msgs::Point &a, const geometry_msgs::Point &b);
	void update();

private:
	Line m_line;
	std::list<geometry_msgs::Point> m_points;
	std::list<std::list<geometry_msgs::Point>::iterator> m_index;
	double m_correl;
};

#endif
