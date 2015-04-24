uint8 EN_COURS = 0  
uint8 LIBRE    = 1  # Il faut attendre l'etat LIBRE, SUCCES ou ECHEC avant de redemander un chemin
uint8 SUCCES  = 2 
uint8 ECHEC  = 3

int16 id
uint8 state
