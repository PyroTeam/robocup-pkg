#include "pathfinder/Point.hpp"

Point::Point()
{
    setPosition(0,0);
    setType(INTERDIT);

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

    m_lowerPoint = NULL;
    m_greaterEqualPoint = NULL;
    m_heapFatherPoint = NULL;
    m_opened = false;
    m_closed = false;

    sm_openListSize = 0;
    sm_closeListSize = 0;
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

void Point::insertInOpenList(Point** openList, bool trueInsert)
{
    // Cas terminal, on insert le point ici
    if((*openList) == NULL)
    {
        m_opened = true;
        (*openList) = this;
        if(trueInsert)
        {
            sm_openListSize++;
        }
        return;
    }

    if(*this < *(*openList))
    {
        // On tente l'insertion sur la branche des inferieurs
        insertInOpenList(&((*openList)->m_lowerPoint), (*openList), trueInsert);
    }
    else
    {
        // On tente l'insertion sur la branche des superieurs ou egaux
        insertInOpenList(&((*openList)->m_greaterEqualPoint), (*openList), trueInsert);
    }
}

void Point::insertInOpenList(Point** openList, Point* father, bool trueInsert)
{
    m_heapFatherPoint = father;
    insertInOpenList(openList, trueInsert);
}

void Point::insertInCloseList()
{
    m_closed = true;
    sm_closeListSize++;
}

void Point::removeFromOpenList(Point** openList)
{
    // - Reconstruction de l'abre binaire
    // Si il n'y a pas de pere
    if(m_heapFatherPoint == NULL)
    {
        // On est le premier point de l'openList
        if(m_greaterEqualPoint != NULL)
        {
            m_greaterEqualPoint->m_heapFatherPoint = m_heapFatherPoint;
            *openList = m_greaterEqualPoint;

            if(m_lowerPoint!=NULL) 
            {
                m_lowerPoint->insertInOpenList(&m_greaterEqualPoint, m_greaterEqualPoint, false); 
            }
        }
        else
        {
            if(m_lowerPoint != NULL)
            {
                m_lowerPoint->m_heapFatherPoint = m_heapFatherPoint;
            }
            *openList = m_lowerPoint;
        }
    }
    // Si il y a un pere
    else 
    {
        // Si plus petit que le pere
        if(*this < *m_heapFatherPoint)
        {
            if(m_greaterEqualPoint != NULL)
            {
                m_greaterEqualPoint->m_heapFatherPoint = m_heapFatherPoint;
                m_heapFatherPoint->m_lowerPoint = m_greaterEqualPoint;

                if(m_lowerPoint!=NULL) 
                {
                    m_lowerPoint->insertInOpenList(&m_greaterEqualPoint, m_greaterEqualPoint, false); 
                }
            }
            else
            {
                if(m_lowerPoint != NULL)
                {
                    m_lowerPoint->m_heapFatherPoint = m_heapFatherPoint;
                }
                m_heapFatherPoint->m_lowerPoint = m_lowerPoint;
            }

        }
        // Si plus grand que le pere
        else 
        {
            if(m_lowerPoint != NULL)
            {
                m_lowerPoint->m_heapFatherPoint = m_heapFatherPoint;
                m_heapFatherPoint->m_greaterEqualPoint = m_lowerPoint;

                if(m_greaterEqualPoint!=NULL) 
                {
                    m_greaterEqualPoint->insertInOpenList(&m_lowerPoint, m_lowerPoint, false); 
                }
            }
            else
            {
                if(m_greaterEqualPoint != NULL)
                {
                    m_greaterEqualPoint->m_heapFatherPoint = m_heapFatherPoint;
                }
                m_heapFatherPoint->m_greaterEqualPoint = m_greaterEqualPoint;
            }
        }
    }

    // - Raz du point
    m_lowerPoint = NULL;
    m_greaterEqualPoint = NULL;
    m_heapFatherPoint = NULL;
    m_opened = false;
    sm_openListSize--;    
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

unsigned int Point::openListSize()
{
    return sm_openListSize;
}

unsigned int Point::closeListSize()
{
    return sm_closeListSize;
}

void Point::printList(Point* openList, int &cpt)
{
    if(openList == NULL)
    {
        std::cout << "Open List is Empty ! " << std::endl;
        return;
    }

    if(openList->m_lowerPoint != NULL)
    {
        printList(openList->m_lowerPoint, cpt);
    }
    
    std::cout << openList->getF() << " (" << openList->getRaw() << ";" << openList->getColumn() << ")" << std::endl;
    std::cout << " \t:Fat: " <<((openList->m_heapFatherPoint)?openList->m_heapFatherPoint->str():"NULL") 
    << " :Low: " << ((openList->m_lowerPoint)?openList->m_lowerPoint->str():"NULL") 
    << " :Gre: " << ((openList->m_greaterEqualPoint)?openList->m_greaterEqualPoint->str():"NULL") << "" << std::endl;
    cpt++;

    if(openList->m_greaterEqualPoint != NULL)
    {
        printList(openList->m_greaterEqualPoint, cpt);
    }
}

int Point::countListRecurs(Point* openList, int &cpt)
{
    if(openList == NULL)
    {
        cpt = 0;
        return cpt;
    }

    if(openList->m_lowerPoint != NULL)
    {
        countListRecurs(openList->m_lowerPoint, cpt);
    }
    cpt++;

    if(openList->m_greaterEqualPoint != NULL)
    {
        countListRecurs(openList->m_greaterEqualPoint, cpt);
    }

    return cpt;
}


int Point::countList(Point* openList)
{
    int count = 0;
    countListRecurs(openList, count);

    return count;
}

void Point::printList2(Point* openList, int &cpt)
{
    if(openList == NULL)
    {
        std::cout << "Open List is Empty ! " << std::endl;
        return;
    }

    std::cout << openList->getF() << " (" << openList->getRaw() << ";" << openList->getColumn() << ")" << std::endl;
    std::cout << " \t:Fat: " <<((openList->m_heapFatherPoint)?openList->m_heapFatherPoint->str():"NULL") 
    << " :Low: " <<openList->m_lowerPoint<<" "<< ((openList->m_lowerPoint)?openList->m_lowerPoint->str():"NULL") 
    << " :Gre: " <<openList->m_greaterEqualPoint<<" "<< ((openList->m_greaterEqualPoint)?openList->m_greaterEqualPoint->str():"NULL") << "" << std::endl;
    cpt++;

    std::cout << "LOW " << openList->m_lowerPoint <<std::endl;
    if(openList->m_lowerPoint != NULL)
    {
        printList2(openList->m_lowerPoint, cpt);
    }
    
    std::cout << "UP " << openList->m_greaterEqualPoint << std::endl;
    if(openList->m_greaterEqualPoint != NULL)
    {
        printList2(openList->m_greaterEqualPoint, cpt);
    }
}

std::string Point::str()
{
    std::cout << getF() <<" ("<< getRaw() <<";"<< getColumn() <<") ";
    
    return "";
}

// Static init
unsigned int Point::sm_openListSize = 0;
unsigned int Point::sm_closeListSize = 0;