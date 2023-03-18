# Vendor constant
uint8 RDT_VENDOR_ID = 56

uint8 vendor_id RDT_VENDOR_ID
uint8 shirt_number 0
uint8 shirt_color 0

# Role constants
uint8 ROLE_RESERVE = 0
uint8 ROLE_FIELDPLAYER = 1
uint8 ROLE_GOALLIE = 2
	
uint8 assigned_role ROLE_RESERVE	# role as assigned by the sideline 
uint8 active_role ROLE_RESERVE 		# role which the robot currently has
uint8 dynamic_role ROLE_RESERVE
uint8 game_state 0
uint16 behavior_state 0
bool control_ball false
bool team_control_ball false
