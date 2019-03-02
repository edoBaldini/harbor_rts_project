#ifndef USER_H
#define USER_H

//------------------------------------------------------------------------------
//	USER FUNCTIONS: manage the interaction with the user. 
//	He can interact with the program through the mouse and keyboard.
// 	
//	Mouse button pressed over a parked ship:
//	-	left awakens a ship
//	-	right adds 200 ms to the parking time of a ship
//	
//	Keyboard keys:
//	-	ESC to terminate the execution
//	-	SPACE BAR to show the routes of the ships in the map
//	-	ENTER to create a new ship, once each 2 seconds.
//------------------------------------------------------------------------------

//	manage the 3 keys allowed to the user of the keyboard:
void botton_pressed();

/*identifies the ship parked in the place clicked by the user.
	If in that place there is no ship, returns -1 */
int find_parked();

//	set the parking_time of the parked ship clicked by the user to now waking it
void woke_up();

//	adds 200 ms to the parking_time of the parked ship clicked by the user
void add_parking_time();

//	initialize the fields of the ship and its route having the indicated id
void initialize_ship(int i);

//	initializes new or reactivates a ship by calling initialize_ship
void init_ship();

// identifies the place id clicked by the user. Otherwise returns -1
int click_place();

#endif