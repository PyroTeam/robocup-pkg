#include "pathfinder/Point.hpp"


Point::Point(float x, float y, typePoint type)
{
    setPosition(x,y);
    setType(type);

    setH(0);
    setG(0);
    setF(0);
    setPointPrec(NULL);
    setRaw(-1);
    setColumn(-1);

    m_opened=false;
    m_closed=false;
    m_lowerPoint=NULL;
    m_greaterEqualPoint=NULL;
    m_heapFatherPoint=NULL;
}

Point::Point(float x, float y, signed int raw, signed int column, typePoint type)
{
    setPosition(x,y);
    setType(type);


    setH(0);
    setG(0);
    setF(0);
    setPointPrec(NULL);
    setRaw(raw);
    setColumn(column);


    m_opened=false;
    m_closed=false;
    m_lowerPoint=NULL;
    m_greaterEqualPoint=NULL;
    m_heapFatherPoint=NULL;
}

Point::~Point()
{
}

int Point::setPosition(float x, float y)
{
    m_x = x;
    m_y = y;

    return 0;
}

int Point::setType(typePoint type)
{
    m_type = type;

    return 0;
}

float Point::getX() const
{
    return m_x;
}

float Point::getY() const
{
    return m_y;
}

typePoint Point::getType() const
{
    return m_type;
}

// A Star
int Point::setPointPrec(Point *pointPrecedent)
{
    m_pointPrecedent = pointPrecedent;
    return 0;
}

int Point::getPointPrec(Point* &pointPrecedent)
{
    pointPrecedent = m_pointPrecedent;
    return 0;
}

float Point::getH() const
{
    return m_h;
}

float Point::getG() const
{
    return m_g;
}

float Point::getF() const
{
    return m_f;
}

void Point::setH(float h)
{
    m_h = h;
}

void Point::setG(float g)
{
    m_g = g;
}

void Point::setF(float f)
{
    m_f = f;
}

signed int Point::getRaw() const
{
    return m_raw;
}

signed int Point::getColumn() const
{
    return m_column;
}

void Point::setRaw(signed int raw)
{
    m_raw = raw;
}

void Point::setColumn(signed int column)
{
    m_column = column;
}

bool Point::isFree()
{
    return (m_type == LIBRE);
}

float Point::distWith(Point const *pointDistant) const
{
    float dist  = 0;
    float distX = 0, distY = 0;

    distX = fabs(pointDistant->getX() - getX());
    distY = fabs(pointDistant->getY() - getY());

    dist = std::sqrt(distX*distX + distY*distY);

    return dist;
}

void Point::reset()
{
    setH(0);
    setG(0);
    setF(0);
    setPointPrec(NULL);
}


// -- Operateurs de comparaison
bool operator<(Point const &p1, Point const &p2)
{
    return p1.getF()<p2.getF();
}
bool operator<=(Point const &p1, Point const &p2)
{
    return p1.getF()<=p2.getF();
}
bool operator>(Point const &p1, Point const &p2)
{
    return p1.getF()>p2.getF();
}
bool operator>=(Point const &p1, Point const &p2)
{
    return p1.getF()>=p2.getF();
}
bool operator==(Point const &p1, Point const &p2)
{
    return (p1.getRaw() == p2.getRaw() && p1.getColumn() == p2.getColumn());
}



// AStar Openset
bool Point::isInOpenList()
{
    return m_opened;
}

bool Point::isInCloseList()
{
    return m_closed;
}

void Point::insertInOpenList(Point *openList)
{
    // Cas terminal, on insert le point ici
    if(openList == NULL)
    {
        m_opened = true;
        return;
    }

    // On sauvegarde le parent (au cas ou on est insere a la prochaine iteration)
    m_heapFatherPoint = openList;
    if(*this < *openList)
    {
        // On tente l'insertion sur la branche des inferieurs
        insertInOpenList(openList->m_lowerPoint);
    }
    else
    {
        // On tente l'insertion sur la branche des superieurs ou egaux
        insertInOpenList(openList->m_greaterEqualPoint);
    }
}

void Point::removeFromOpenList()
{
    // Reconstruction de l'arbre binaire
    m_heapFatherPoint->m_greaterEqualPoint = m_lowerPoint;
    m_greaterEqualPoint->insertInOpenList(m_lowerPoint);

    // RAZ du point
    m_lowerPoint = NULL;
    m_greaterEqualPoint = NULL;
    m_heapFatherPoint = NULL;
    m_opened = false;
    m_closed = true;
}

Point* Point::getLowerFromOpenList(Point *openList)
{
    if(openList!= NULL && openList->m_lowerPoint != NULL)
    {
        return getLowerFromOpenList(openList->m_lowerPoint);
    }
    else if(openList!= NULL && openList->m_lowerPoint == NULL)
    {
        return openList;
    }
    else
    {
        return NULL;
    }
}

bool Point::openListIsEmpty(Point *openList)
{
    if(openList!= NULL)
    {
        return false;
    }
    else
    {
        return true;
    }
}