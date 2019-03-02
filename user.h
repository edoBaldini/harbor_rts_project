#ifndef USER_H
#define USER_H

//------------------------------------------------------------------------------
//	USER FUNCTIONS
//------------------------------------------------------------------------------

/*manage the 3 type of input allowed to the user by the keyboard:
	ESC: 		put the end global variable to true and terminating the program
	ENTER:		invokes init_ship, allowed once each 2 seconds
	BAR SPACE: 	set the globa variable show_routes to true, showing the routes*/
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