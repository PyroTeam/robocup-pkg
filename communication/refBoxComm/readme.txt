La package refBoxComm contient
 - le noeud refBoxComm pour la communication avec la referee Box
       commande : $ rosrun refBoxComm refBoxComm

 - des scripts de test pour verifier chaques fonctionnalités du noeud
    * test_reportMachine.py permet de tester le service de déclaration de machine 
       commande : $ rosrun refBoxComm test_reportMachine.py [nom] [type] 

    * test_gameStateListener.py permet de tester le topic GameState
       commande : $ rosrun refBoxComm test_gameStateListener.py
    * test_ExplorationInfoListener.py permet de tester le topic ExplorationInfo
       commande : $ rosrun refBoxComm test_ExplorationInfoListener.py

Liste des topics et services disponibles via le noeud refBoxComm

 - Topic : 

/GameState
	time game_time

	uint8 INIT       = 0
	uint8 WAIT_START = 1
	uint8 RUNNING    = 2
	uint8 PAUSED     = 3

	uint8 state

	uint16 PRE_GAME    =  0
	uint16 SETUP       = 10
	uint16 EXPLORATION = 20
	uint16 PRODUCTION  = 30
	uint16 POST_GAME   = 40
	uint16 OPEN_CHALLENGE         = 1000
	uint16 NAVIGATION_CHALLENGE   = 1001
	uint16 WHACK_A_MOLE_CHALLENGE = 1002

	uint16 phase
	uint32 points


/ExplorationInfo
	ExplorationSignal[] signals
		string type
		LightSpec[] lights
			uint8 RED    = 0
			uint8 YELLOW = 1
			uint8 GREEN  = 2

			uint8 OFF    = 0
			uint8 ON     = 1
			uint8 BLINK  = 2

			uint8 color
			uint8 state
	ExplorationMachine[] machines
		string name
		geometry_msgs/Pose2D pose
			float64 x
			float64 y
			float64 theta

- Services : 

/ReportMachine
	string name
	string type
	---




