#include "pathfinder/Objet.hpp"


	Objet::Objet(typeObjet type, float x, float y)
	{
		setType(type);
		setPosition(x, y);
		_phi = 0;
	}

	Objet::Objet(typeObjet type, float x, float y, int phi)
	{
		setType(type);
		setPosition(x, y, phi);
	}

	Objet::~Objet()
	{
	}

	int Objet::setPosition(float x, float y)
	{
		_x = x;
		_y = y;

		return 0;
	}

	int Objet::setPosition(float x, float y, int phi)
	{	
		setPosition(x, y);

		// Angle compris entre -180 et 180 degrés
		_phi = (phi%360);
        if (_phi > 180)
        {
            _phi = _phi-360;
        }
        else if(_phi <= -180)
        {
            _phi = _phi+360;
        }

		return 0;
	}

	int Objet::setType(typeObjet type)
	{	
		_type = type;

		return 0;
	}

	float Objet::getX() const
	{
		return _x;
	}

	float Objet::getY() const
	{
		return _y;
	}

	int Objet::getPhi() const
	{
		return _phi;
	}
