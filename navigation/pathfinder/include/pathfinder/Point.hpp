#ifndef _PATHFINDER__POINT_H_
#define _PATHFINDER__POINT_H_

#include <cmath>
#include <cstddef>

enum typePoint{LIBRE=1,OCCUPE_AMI,OCCUPE_ADVERSAIRE,INTERDIT};

class Point
{
public:
    Point(float x, float y, typePoint type = LIBRE);
    Point(float x, float y, signed int raw, signed int column, typePoint type = LIBRE);
    ~Point();

    int setPosition(float x, float y);
    int setType(typePoint type);
    float getX() const;
    float getY() const;
    typePoint getType() const;

    // AStar
    int setPointPrec(Point *pointPrecedent);
    int getPointPrec(Point *&pointPrecedent);
    float getH() const;
    float getG() const;
    float getF() const;
    void setH(float h);
    void setG(float g);
    void setF(float f);
    signed int getRaw() const;
    signed int getColumn() const;
    void setRaw(signed int raw);
    void setColumn(signed int column);
    bool isFree();
    float distWith(Point const *pointDistant) const;
    void reset();

    // AStar Openset
    bool isInOpenList();
    bool isInCloseList();
    void insertInOpenList(Point *openList);
    void removeFromOpenList();
    Point* getLowerFromOpenList(Point *openList);
    bool openListIsEmpty(Point *openList);

private:
    // Position du point
    float m_x;
    float m_y;
    signed int m_raw;
    signed int m_column;

    // Type
    typePoint m_type;

    // AStar
    Point *m_pointPrecedent;
    float m_h;  // Distance estimee jusqu'a arrivee (heuristic)
    float m_g;  // Distance connue depuis depart
    float m_f;  // Distance du chemin via ce point -> connue depuis depart + estimee jusqu'a arrivee (g+h)

    // AStar Openset
    bool m_opened;
    bool m_closed;
    Point* m_lowerPoint;
    Point* m_greaterEqualPoint;
    Point* m_heapFatherPoint;

};

// Surcharge d'opérateurs de comparaison
    bool operator<(Point const& p1, Point const& p2);
    bool operator==(Point const& p1, Point const& p2);

#endif  // _PATHFINDER__POINT_H_
