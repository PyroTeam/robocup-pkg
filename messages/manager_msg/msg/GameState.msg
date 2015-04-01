time game_time

uint8 INIT = 0
uint8 WAIT_START = 1
uint8 RUNNING = 2
uint8 PAUSED = 3

uint8 state

uint16 PRE_GAME = 0
uint16 SETUP = 10
uint16 EXPLORATION = 20
uint16 PRODUCTION = 30
uint16 POST_GAME = 40

uint16 OPEN_CHALLENGE = 1000
uint16 NAVIGATION_CHALLENGE = 1001
uint16 WHACK_A_MOLE_CHALLENGE = 1002

uint16 phase

uint32 points
