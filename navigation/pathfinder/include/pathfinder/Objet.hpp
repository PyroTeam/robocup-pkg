#ifndef _HEADER_OBJET_
#define _HEADER_OBJET_

enum typeObjet{MACHINE,ROBOTINO,DELIVERY,RECYCLING};

class Objet
{
public:
	Objet(typeObjet type, float x, float y);
	Objet(typeObjet type, float x, float y,int phi);
	~Objet();
	int setPosition(float x, float y);
	int setPosition(float x, float y, int phi);
	int setType(typeObjet type);
	float getX() const;
	float getY() const;
	int getPhi() const;

private:
	// Position de l'objet
	float _x;
	float _y;
	int _phi;	

	// Type
	typeObjet _type;
};

#endif	// _HEADER_OBJET_