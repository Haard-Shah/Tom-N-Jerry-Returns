#include <stdint.h>
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <util/delay.h>
#include <cpu_speed.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <macros.h>
#include <graphics.h>
#include <lcd_model.h>
#include "lcd.h"
#include "usb_serial.h"
#include "cab202_adc.h"
#include <math.h> //only using the cos and sin functions so can get them and add them to helper functions if it gets too loaded on teensy


#define FREQ     (8000000.0)
#define PRESCALE3 (8.0)  //  for a Freq of 31.25Khz
#define SuperMode_MAX 312500
#define Trap_timer_MAX 5.7220459
#define Cheese_timer_MAX 3.8146973
#define Milk_potion_timer_MAX 9.5367432

#define STATUS_BAR_HEIGHT 9
#define GAMEPLAY_X (LCD_X - 2) // -2 for border
#define GAMEPLAY_Y (LCD_Y - 2) // -2 for border
#define M_PI 3.14159265358979323846

#define SQRT(x,y) sqrt(x*x + y*y)
#define calcDistanceBetween2Points(x1, y1, x2, y2) SQRT((x1-x2), (y1-y2)) // Calculates the Euclidean distance between provide x1, y1 and x2, y2 coordiantes.
#define absolute(x) (x<0 ? x*-1 : x)
#define send_msg(msg) send_msg_(msg, "\r\n") // wrapper for send_msg_() with new line and return passed in as second arg
#define BIT_IS_ON(x, y) read_pixel(x, y) // wrapper function for read_pixel()
#define setup_LED(x) SET_BIT(DDRB, x + 2)
#define turn_on_LED(x) SET_BIT(PORTB, x + 2)
#define turn_off_LED(x) CLEAR_BIT(PORTB, x + 2)
#define WRAP(y_next, range) fmod((fmod(y_next, range) + range), range) // Two way wrapper for positive and negative values Credit: Mauro Bringolf Link:https://dev.to/maurobringolf/a-neat-trick-to-compute-modulo-of-negative-numbers-111e
#define setup_firework(firework_index) move_weapon(Jerry.x, Jerry.y, firework_index) 
#define draw_door() draw_object(&Door, door)
#define draw_milk_potion() draw_object(&milk_potion, potion)
#define draw_cheeses() draw_objects(cheeses, cheese, MAX_cheeses)
#define draw_traps() draw_objects(traps, trap, MAX_traps)
#define draw_bitmap(bitmap, x, y) draw_bitmap_(bitmap, x, y, FG_COLOUR)
#define clear_bitmap(bitmap, x, y) draw_bitmap_(bitmap, x, y, BG_COLOUR)
#define true 1
#define false 0

#define SCORE (game_cheeseEaten + game_firework_hit + carryOver_score)

#define MAX_WALLS 4
#define MAX_cheeses 5
#define MAX_traps 5
#define MAX_FIREWORKS 20
#define WEAPON_SPEED 0.6
#define cheese_width (5)
#define cheese_height (5)
#define trap_width (5)
#define trap_height (5)
#define Jerry_width (SuperMode ? 8 : 5)
#define Jerry_height (SuperMode ? 8 : 7)
// #define Tom_width (8)
// #define Tom_height (8)
#define door_width (5)
#define door_height (8)
#define milk_potion_width (7)
#define milk_potion_height (7)
#define MAX_JERRY_small_SPEED 1.0
#define MAX_WALL_SPEED 1.0
#define MAX_LVL 2
#define TOLERANCE 0.00000000000000000  // (10e-8)
#define MAX_CHECKS (10)

// Struct Definitions, Game Objects
struct bounding_box
{
    int top;
    int left;
    int bottom;
    int right;
};

struct player {
    double x;
    double y;
    double dx;
    double dy;
    double reset_x;
    double reset_y;
    struct bounding_box bounding_box;
};

struct object{
    int x;
    int y;
    uint8_t visible;
    struct bounding_box bounding_box;
};

struct weapon {
    double x;
    double y;
    uint8_t visible;
};

struct wall
{
    double x1;
    double y1; 
    double x2;
    double y2;
    double dx;
    double dy;
};


// Global Variabels
uint8_t filler[8] = {
    0b11000000,
    0b11000000,
    0b11000000,
    0b11000000,
    0b11000000,
    0b11000000,
    0b11000000,
    0b11000000,
};

uint8_t heart[8] = {
    0b10111011,
    0b00010001,
    0b01000001,
    0b10000011,
    0b11000111,
    0b11000111,
    0b11101111,
    0b11101111,

};

uint8_t Tom_width = 8;
uint8_t Tom_height = 8;

uint8_t tom_small[8] = {
    0b10001000,
    0b10001000,
    0b01010000,
    0b10001000,
    0b01010000,
    0b00100000,
    0b01110000,
    0b00000000,
};

uint8_t tom[8] = {
    0b10000001,
    0b11000011,
    0b01010101,
    0b10000001,
    0b01011010,
    0b00100100,
    0b00011000,
    0b00100100,
};

// uint8_t Jerry_width = 5;
// uint8_t Jerry_height = 7;

uint8_t jerry[8] ={
    0b11111000,
    0b00100000,
    0b00100000,
    0b00100000,
    0b00100000,
    0b10100000,
    0b01000000,
    0b00000000,
};

uint8_t jerry_large[8] = {
    0b00001000,
    0b01010000,
    0b00111010,
    0b01000101,
    0b01101001,
    0b10010001,
    0b01000110,
    0b00111000,
};

uint8_t cheese[8] = {
    0b11011000,
    0b01101000,
    0b00111000,
    0b00011000,
    0b00001000,
    0b00000000,
    0b00000000,
    0b00000000,
};

uint8_t trap[8] = {
    0b11111000,
    0b10101000,
    0b11111000,
    0b10001000,
    0b11111000,
    0b00000000,
    0b00000000,
    0b00000000,
};

uint8_t door[8] = {
    0b11111000,
    0b10001000,
    0b10001000,
    0b10001000,
    0b10001000,
    0b10001000,
    0b10001000,
    0b11111000,
};

uint8_t potion[8] = {
    0b00111000,
    0b00101000,
    0b00101000,
    0b01000100,
    0b10000010,
    0b10000010,
    0b01111100,
    0b00000000,
};

struct wall walls[MAX_WALLS];
struct player Jerry;
struct player Tom;
struct object cheeses[MAX_cheeses];
struct object traps[MAX_traps];
struct object milk_potion;
struct object Door;
struct weapon fireworks[MAX_FIREWORKS];

int game_lvl;
int game_lives;
int game_cheeseEaten;
int game_min;
double game_sec;
uint8_t game_paused;
uint8_t game_gameOver;
int game_cheeses;
int game_traps;
int game_pause_time;
int game_firework_hit;
uint8_t carryOver_score = 0;

uint8_t game_started = false;
uint8_t paused_history = false;
int rnd; 
double wall_speed, jerry_small_speed;
uint8_t walls_index = 0;
uint8_t downwards = 0;

uint8_t right_SW_history = 0;
uint8_t right_SW_pressed = 0;

uint8_t left_SW_history = 0;
uint8_t left_SW_pressed = 0;

//TODO: for memory management change these to static single history vairable and replace the pin in the check_joystick()
uint8_t up_joystick_history = 0;
uint8_t down_joystick_history = 0;
uint8_t right_joystick_history = 0;
uint8_t left_joystick_history = 0;
uint8_t center_joystick_history = 0;


// Volatile Variables
volatile int overflow_counter3_time = 0;
volatile int overflow_counter1_cheese = 0;
volatile int overflow_coounter2_traps = 0;
volatile int overflow_counter2_milk_potion = 0;
volatile int overflow_counter2_SuperJerry_small = 0;
volatile int cheeseTimerTemp = 0;
volatile int trapTimerTemp = 0;
volatile int milkPotionTimerTemp = 0;
volatile int SuperMode = 0;
volatile int dutyCycle = 8;
volatile uint8_t ticker = 0;


//Forawrd Declerations

void draw_players();
void draw_bitmap_(uint8_t bitmap[], int x, int y, colour_t color);
int isValidLocation2(uint8_t bitmap[], int x, int y, int width, int height);
void pause_game();
void next_level();
int clear_of_wall_collisions_x(int x, int y);
int clear_of_wall_collisions_y(int x, int y);
void draw_walls();
void Game_Over();

// -------------------------------------------------
// Helper functions.
// -------------------------------------------------
void draw_formatted(int x, int y, char * buffer, int buffer_size,  colour_t color, const char * format, ...) {
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, buffer_size, format, args);
	draw_string(x, y, buffer, color);
}

/* 
Sends parssed message to Serial Connection.

Credit: Prof. Lewis LiveDemo.c WK12.
*/
void usb_serial_send(char * message) {
	// Cast to avoid "error: pointer targets in passing argument 1 
	//	of 'usb_serial_write' differ in signedness"
	usb_serial_write((uint8_t *) message, strlen(message));
}

/* 
Functions reads the sequencial characters in the input stream uptil the newline characters and stores the incoming input into the parssed pointer.

Credit: Prof. Lewis LiveDemo.c WK12.
*/
void usb_serial_read_string(char * message){
   int c = 0;
   int buffer_count = 0;
      
   while (c != '\n' && c!=-1) // check for new line as well as empty characters
    {
        c = usb_serial_getchar();
        message[buffer_count] = c;
        buffer_count++;
    }
}

/* wrapper functions to send messages via serial connection. */
void send_msg_(char *msg, char *msg2)
{
    char *buf = msg;
    usb_serial_send(buf);
    char *n = msg2;
    usb_serial_send(n);
}

/* wrapper functions to send doubles via serial connection. */
void send_double(double d)
{
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "%lf", d);
    send_msg(buffer);
}

/* Read potentiometers and convert to a range value.  */
void read_potentiometers()
{
    //Read the potentiomenters
    int right_adc = adc_read(1);
	int left_adc = adc_read(0);
    
    //convert to correct value
    if (right_adc > 510 ){ wall_speed = ((double)right_adc - 510.0)/513.0; }
    else if (right_adc > 507) { wall_speed = 0; } // Create a dead zone for ease of movement
    else { wall_speed = ((double)right_adc - 506.0)/506.0;}

    wall_speed = wall_speed * MAX_WALL_SPEED;

    //convert to correct value 
    jerry_small_speed = ((1023.0 - (double)left_adc)/1023.0) * MAX_JERRY_small_SPEED;    
}

// --------------------------- INTERUPPT FUNCTIONS -----------------

void debounce_right_switch()
{
    // Perform debouncing for the first switch
    uint8_t mask = 0b00001111;

    right_SW_history = ((right_SW_history<<1) & mask) | BIT_VALUE(PINF, 5); // shift the entire history state left by 1 and store the new value of the button's state 

    if (right_SW_history == mask) 
    { 
        while(BIT_IS_SET(PINF, 5)) //Hold it while its still on so it doesn't keep toggling the game_pause state
        right_SW_pressed = 1; //register the press
    } // if the switch has been consistantly on for last 3 passes it must be on (i.e. right_SW_pressed)
    else if (right_SW_history == 0) 
    { 
        right_SW_pressed= 0;
    } // Similarly if it has been off for past 3 passes then it must be off
}

void debounce_left_switch()
{
    // Perform debouncing for the first switch
    uint8_t mask = 0b00000111;

    left_SW_history = ((left_SW_history<<1) & mask) | BIT_VALUE(PINF, 6); // shift the entire history state left by 1 and store the new value of the button's state 

    if (left_SW_history == mask) { 
        while(BIT_IS_SET(PINF, 6)) // Hold until button is lifted
        Game_Over(); 
    } // if the switch has been consistantly on for last 3 passes it must be on (i.e. left_SW_pressed)
}

ISR(TIMER0_OVF_vect)
{
    if (SuperMode && overflow_counter2_SuperJerry_small <= SuperMode_MAX)
    {
        // PWM the LED's 
        if (ticker < dutyCycle) 
        {
            turn_on_LED(0); turn_on_LED(1);
        }
        else
        {
            turn_off_LED(0); turn_off_LED(1);
        }
        ticker++;            
        overflow_counter2_SuperJerry_small++;
    }
    else
    {
        overflow_counter2_SuperJerry_small = 0;
        SuperMode = false;
        turn_off_LED(0); turn_off_LED(1);// Ensureing the lights are off all the time
        // TODO: reset the bit mask back to normal jerry
        // jerry = jerry_small;
        // Jerry_width = 5; Jerry_height = 7;
    }

    debounce_left_switch();
}

ISR(TIMER3_OVF_vect)
{
    if(game_started) overflow_counter3_time++; // Increment the overflow_counter3_time 

    debounce_right_switch();
    // debounce_left_switch();
}
 
char check_joystick()
{
    //Check all of the joysticks
    uint8_t mask = 0b00000011;

    //Debounce and check state of 5 way joystick 
    // Right Switch
    right_joystick_history = (((right_joystick_history)<<1) & mask) | BIT_VALUE(PIND, 0);
    if ((right_joystick_history) == mask) { return 'd'; } // if the switch has been consistantly on for last 3 passes it must be on (i.e. right_SW_pressed)

    // Down Switch
    down_joystick_history = (((down_joystick_history)<<1) & mask) | BIT_VALUE(PINB, 7);
    if ((down_joystick_history) == mask) { return 's'; } // if the switch has been consistantly on for last 3 passes it must be on (i.e. right_SW_pressed)
    
    // Left Switch
    left_joystick_history = (((left_joystick_history)<<1) & mask) | BIT_VALUE(PINB, 1);
    if ((left_joystick_history) == mask) { return 'a'; } // if the switch has been consistantly on for last 3 passes it must be on (i.e. right_SW_pressed)

    // Up Switch
    up_joystick_history = (((up_joystick_history)<<1) & mask) | BIT_VALUE(PIND, 1);
    if ((up_joystick_history) == mask) { return 'w'; } // if the switch has been consistantly on for last 3 passes it must be on (i.e. right_SW_pressed)

    // Center Switch
    center_joystick_history = (((center_joystick_history)<<1) & mask) | BIT_VALUE(PINB, 0);
    if ((center_joystick_history) == mask) { return 'f'; } // if the switch has been consistantly on for last 3 passes it must be on (i.e. right_SW_pressed)

    else { return ' '; }
}

ISR(TIMER1_OVF_vect)
{
    // increment cheese and mouse trap counters
    if (!game_paused) 
    {
        overflow_counter1_cheese++;
        overflow_coounter2_traps++;
        overflow_counter2_milk_potion++;
    }
}

// ............................ SETUP FUNCITONS .............................
/* Setup bounding_box properties for the parssed object. */
void Add_bounding_box(struct bounding_box *BB, int top, int left, int height, int width)
{
    BB->top = top;
    BB->left = left;
    BB->bottom = (top + height);
    BB->right = (left + width);
}


/* setup 5 way joystick for use.  */
void setup_joystick()
{
    //enable joystick buttons
    SET_INPUT(DDRD, 1); // up
	SET_INPUT(DDRB, 1); // left
	SET_INPUT(DDRD, 0); // right
	SET_INPUT(DDRB, 7); // down
    SET_INPUT(DDRB, 0); // center
}


/* Setup Timer 0 8 Bit timer with overflow enabled. */
void setup_timer0()
{
    TCCR0A = 0; // Enable normal mode
	TCCR0B = 1; // to get a overflow freq of about 30 times per second
    TIMSK0 = 1; // Enable Timer overflow interrupt for Timer 0
}

/* Setup Timer 0 8 Bit timer with overflow enabled. */
void setup_timer1()
{
    TCCR1A = 0; // Enable normal mode
	TCCR1B = 3; // to get a overflow freq of about 30 times per second
    TIMSK1 = 1; // Enable Timer overflow interrupt for Timer 0
}

/* Setup Timer 4 10 Bit timer with overflow enabled. */
void setup_timer3()
{
    TCCR3A = 0; // Enable normal mode
	TCCR3B = 2; // to get a overflow freq of about 30 times per second
    TIMSK3 = 1; // Enable Timer overflow interrupt for Timer 0
}

/* Defines the corrdinates of the cheese respawn location. */
void setup_cheese(int cheese_index)
{
    // FIXME: The while conditions needs to be fixed so that it check for all the on pixels of cheese bitmap and not just the the starting pixel
    int counter = 0;
     do
        {
            cheeses[cheese_index].x = 1 + (rand() % (GAMEPLAY_X - 5));
            cheeses[cheese_index].y = STATUS_BAR_HEIGHT + (rand() % (GAMEPLAY_Y - STATUS_BAR_HEIGHT - 4));
            counter++;

        } while (!isValidLocation2(cheese, cheeses[cheese_index].x, cheeses[cheese_index].y, cheese_width, cheese_height) && counter < MAX_CHECKS); 

        // store the bounding box properties
        Add_bounding_box(&cheeses[cheese_index].bounding_box, cheeses[cheese_index].y, cheeses[cheese_index].x, cheese_height, cheese_width);
}


void move_weapon(double current_x, double current_y, int weapon_index)
{
    //(NOTE: wrt == with respect to)

    //Get a unit vector of Tom's displacement wrt current location 
    double theta;

    if(absolute(Tom.x - current_x) < 0.002 && absolute(Tom.y - current_y) < 0.002) // Putting a guards condition as the movement the magnitude of the step is based on the change in each direction. So if the change is very small the firework has indeed hit TOM
    {
        fireworks[weapon_index].x = Tom.x;
        fireworks[weapon_index].y = Tom.y;
        return;
    }
    // Set a gaurd statement so if dx does == 0 then it is set to infinity which will equate to near 0 when divided
    else if(absolute(Tom.x - current_x) == 0)
    {
        // Weapon is perpendicular to Tom so (dy/dx) == NaN as (dx == 0) therefore setting the theta manually to avoid segmentation fault
        if(Tom.y <= current_y) theta = M_PI / 2;  // 90 degrees Tom is perpendicularly above the weapon
        else theta = M_PI * (3/2); // 270 degrees Tom is perpendiculary below the weapon
    }
    else
    {
        double dy = absolute(Tom.y - current_y);
        double dx = absolute(Tom.x - current_x);
        theta = atan( dy / dx); //absoluteolute unit vector. In this case the magnitute of the unit vector is just 1 as it moves 1 pixel per step so absoluteolute unit vector should be fine for x and y calculations
    } 

    //Find out where Tom is in regards to current location and fire weapon accordingly    
    // dx = speed * cos(theta);  ->  dx = (1) * cos(theta)   ->   dx = cos(theta)  
    // SO, x = current_x + dx -> x = current_x + cos(theta)
     if (Tom.x < current_x ) fireworks[weapon_index].x = current_x - (WEAPON_SPEED * (GAMEPLAY_X/GAMEPLAY_Y) * cos(theta)); // So Tom is to the left of current location 
     else fireworks[weapon_index].x = current_x + (WEAPON_SPEED * (GAMEPLAY_X/GAMEPLAY_Y ) * cos(theta)); // Tom is to the right of current location
    
    // SIMILARLY dy = speed * sin(theta);  ->  dy = (1) * sin(theta)   ->   dy = sin(theta)      
    // SO, y = current_y + dx -> y = current_y + sin(theta)
     if (Tom.y < current_y) fireworks[weapon_index].y = current_y - (WEAPON_SPEED * sin(theta)); // Tom is above current location
     else fireworks[weapon_index].y = current_y + (WEAPON_SPEED *  sin(theta)); // Tom is below current location
}

/* Setup door's location and bounding box properties for door. */
void setup_door()
{
    int x, y;
    int counter = 0;
    do
    {
        x = 1 + (rand() % (GAMEPLAY_X - door_width));
        y = STATUS_BAR_HEIGHT + 1 + (rand() % (GAMEPLAY_Y - STATUS_BAR_HEIGHT - door_height - 1));
        counter++;
    } while (!isValidLocation2(door, Door.x, Door.y, door_width, door_height) && counter < MAX_CHECKS);

    Door.x = x;
    Door.y = y;

    //Bounding box
    Add_bounding_box(&Door.bounding_box, y, x, door_height, door_width);
}

/* Setup Milk potion's location and its bounding box properties at current tom's location. */
void setup_milk_potion()
{
    milk_potion.x = Tom.x;
    milk_potion.y = Tom.y;

    //Bounding Box
    Add_bounding_box(&milk_potion.bounding_box, milk_potion.y, milk_potion.x, milk_potion_height, milk_potion_width);   
}

/* Assign x,y reset_x,rest_y and bounding box properties of parssed player's . */
void setup_player(struct player *player, int x, int y, int width, int height)
{
    player->x = (double)x;
    player->y = (double)y;
    player->reset_x = (double)x;
    player->reset_y = (double)y;

    // store the bounding box properties
    Add_bounding_box(&(player->bounding_box), player->y, player->x, height, width);
}

/* Calculates the perpendicular graidient (dx, dy) for a line given its x1,y1 and x2,y2 coordinates. */
void calc_perpendicular_gradient(int wall_index)
{
    double delta_x, delta_y;
    delta_x = walls[wall_index].x2 - walls[wall_index].x1;

    send_msg("delta x"); send_double(delta_x);

    delta_y = walls[wall_index].y2 - walls[wall_index].y1;

    send_msg("delta y"); send_double(delta_y);

    if (delta_x == 0) // vertical line only move horizontally
    {
        walls[wall_index].dx = delta_y;
        walls[wall_index].dy = 0;
    }
    else if (delta_y == 0) // horizontal line only move in vertically
    {
        walls[wall_index].dx = 0;
        walls[wall_index].dy = delta_x;
    }
    else // wall with a gradient
    {
        double graident = delta_y/delta_x;

        double perpendicualr_gradient_arg = atan(-1/graident);

        walls[wall_index].dx = cos(perpendicualr_gradient_arg);
        walls[wall_index].dy = sin(perpendicualr_gradient_arg);
    }
    
    send_msg("dx"); send_double(walls[wall_index].dx);
    send_msg("dy"); send_double(walls[wall_index].dy);

}

/* Assing values to Wall struct object. */
void setup_wall(int wall_index, int x1, int y1, int x2, int y2)
{
    walls[wall_index].x1 = (double)x1;
    walls[wall_index].y1 = (double)y1;
    walls[wall_index].x2 = (double)x2;
    walls[wall_index].y2 = (double)y2;
}

//-------------------------------------------------------
/* Checks if the two parssed bounding boxes overlap, Hence collide with one another. */
int collided(struct bounding_box *A, struct bounding_box *B)
{
    //Check to see if the colliders are "lined up" on the X-axis 
    if( (A->right > B->left ) && (B->right > A->left) ){

        //Check to see if the colliders are also "lined up" on the Y-axis
        if( (A->top < B->bottom) && (B->top < A->bottom) ) return true; // COLLISION DETECTED        
    }
    return false;//NO COLLISION DETECTED
}

/* Display welcome sreen of the game and wait until for user input to determine next game state.  */
void show_welcome_screen()
{
    // Display the welcome screen 
    draw_string(18, 5, "Haard Shah", FG_COLOUR);
    draw_string(20, 15, "n10235779", FG_COLOUR);
    draw_string(12, 25, " Tom N Jerry ", BG_COLOUR);

    show_screen();

    while( right_SW_pressed == 0 ) 
    {
        char buffer[50];
        itoa(right_SW_pressed, buffer,10);
        
        char *msg = buffer;
        usb_serial_send(msg);
        rnd++;

        _delay_ms(100);

        // Wait until right switch is pressed - not using usb for the weaiting as well as the usb is need from lvl 2 onwards
    }
    clear_screen();
    show_screen();
    game_started = true;

    send_msg("Game Began ...");   
}

/* sets up cheese, traps, firework, milk potion and door for game play. Disable their visibility. */
void initialise_game_objects()
{
    walls_index = 0; //reset the number of walls 

    //initialise cheeses and make them invisible  AND make the traps inviisible 
    for(int i = 0; i < MAX_cheeses; i++) 
    {
        if (cheeses[i].visible) clear_bitmap(cheese, cheeses[i].x, cheeses[i].y);
        setup_cheese(i);
        cheeses[i].visible = false;

        if (traps[i].visible) clear_bitmap(trap, traps[i].x, traps[i].y);
        traps[i].visible = false;
    }

    for (int i = 0; i < MAX_FIREWORKS; i++) {
        if (fireworks[i].visible) draw_pixel(fireworks[i].x, fireworks[i].y, BG_COLOUR);
        fireworks[i].visible = false; //Make all the fireworks invisible at the start
    }
    if (Door.visible) clear_bitmap(door, Door.x, Door.y);
    Door.visible = false; // deactivate door
    
    if (milk_potion.visible) clear_bitmap(potion, milk_potion.x, milk_potion.y);    
    milk_potion.visible = false; // deactivate milk potion
}

/* Initialise game state including game variables; lvl, lives, score and time. */
void initialise_game()
{
    game_lvl = 1;
    game_lives = 5;
    game_cheeseEaten = 0;
    game_firework_hit = 0;
    game_min = 0;
    game_sec = 0;
    game_paused = false;
    game_gameOver = false;
    game_cheeses = 0;
    game_traps = 0;
    Door.visible = false;
}

void update_time()
{
    game_sec = ( overflow_counter3_time * 65535.0 + TCNT3 ) * PRESCALE3  / FREQ; // store time in seconds
	
    // update time accordingly if seconds is >= 60
	if (game_sec >= 60.00) {
		overflow_counter3_time = 0;
		game_min++;
        game_sec = 0.0;    
	}
    
    // Update the game display to reflect time changes
    char buffer[80];
    draw_bitmap(filler, 56, 0);
    draw_formatted(58,0, buffer, sizeof(buffer), BG_COLOUR, "%.2d:%.2d", game_min, (int)game_sec);
    show_screen();

    //FIXME: if the score goes over 9 the time and score overlap. 
}

//TODO: add description 
void initialise_tom_movement()
{
    double direction = rand() * M_PI * 2 / RAND_MAX;
    float speed = ( (50.0 + (double)(rand() % 30)) /100.0) * MAX_JERRY_small_SPEED; //Moding to 80% jerry's speed 
    // float speed = 0.8 * MAX_JERRY_small_SPEED;

    Tom.dx = speed * cos(direction);
    Tom.dy = speed * sin(direction);
}

/* 
Initialise player positions for the game and display the players on the game_
*/
void setup_players()
{
    setup_player(&Jerry, 1, (STATUS_BAR_HEIGHT + 1), Jerry_width, Jerry_height);

    setup_player(&Tom, (GAMEPLAY_X - Tom_height), (GAMEPLAY_Y - Tom_width), Tom_width, Tom_height);
    initialise_tom_movement();

    draw_players();
}

/* Resets the passered player's location back to its original x and y coordinate. */
void reset_player(struct player *player, int width, int height)
{
    // reset the location
    player->x = player->reset_x; //TODO: try *player.reset_x
    player->y = player->reset_y;

    // update the bounding box properties
    Add_bounding_box(&(player->bounding_box), player->y, player->x, height, width);
}

/* Reset's both players to their original location. */
void reset_players()
{
    // Jerry_small
    clear_bitmap((SuperMode ? jerry_large : jerry), round(Jerry.x), round(Jerry.y));
    
    reset_player(&Jerry, Jerry_width, Jerry_height);
    // Tom
    clear_bitmap(tom, round(Tom.x), round(Tom.y));
    reset_player(&Tom, Tom_width, Tom_height);
    
    draw_players();
}

/* Setup walls for level 1 of the game. */
void setup_walls()
{

    // setup all of the walls [Level 1 only]
    setup_wall(0, 18, 15, 13, 25);   

    //
    setup_wall(1, 25, 35, 25, 45);

    // walls[1].dx = walls[1].y2 - walls[1].y1;
    // walls[1].dy = 0;
    // calc_perpendicular_gradient(1);

    //
    setup_wall(2, 45, 10, 60, 10);
    // walls[2].dx = 0;
    // walls[2].dy = walls[2].x2 - walls[2].x1;
    
    //
    setup_wall(3, 58, 25, 72, 30);
    // walls[3].dx = walls[3].x1 - walls[3].x2;
    // walls[3].dy = walls[3].y2 - walls[3].y1;

    walls_index = 3;
    draw_walls();
}

//TODO: Add Description
void read_usb_inputfile()
{
    uint8_t file_provided = false;
    send_double(usb_serial_available());
    if (usb_serial_available())
    {
        char usb_buffer[32];
        int blank_input_counter = 0;
        
        while (blank_input_counter < 10)
        {
            int c = usb_serial_getchar(); // read the first character
            switch (c)
            {
                case 'T':
                    usb_serial_read_string(usb_buffer);
                    int x, y; x = 0; y = 0;
                    sscanf(usb_buffer, "%d %d", &x, &y);
                    setup_player(&Tom, x, y, Tom_width, Tom_height);   
                    send_msg("Tom position defined!");
                    file_provided = true;
                    break;
                
                case 'J':
                    usb_serial_read_string(usb_buffer);
                    int x1, y1; x1 = 0; y1 = 0;
                    sscanf(usb_buffer, "%d %d", &x1, &y1);
                    setup_player(&Jerry, x1, y1, Jerry_width, Jerry_height);
                    send_msg("Jerry_small position defined!");
                    file_provided = true;
                    break;
                
                case 'W':
                    usb_serial_read_string(usb_buffer);
                    
                    if (walls_index < MAX_WALLS)
                    {
                        int w1 ,w2, w3, w4;
                        sscanf(usb_buffer, "%d %d %d %d", &w1, &w2, &w3, &w4);
                       setup_wall(walls_index, w1, w2, w3, w4);
                        walls_index++;
                        file_provided = true;
                        send_msg("Wall Setup!");
                    }                    
                    break;
                
                case 0:
                case -1:
                    blank_input_counter++;
                    break;
                default:
                    break;
            }
        }
    }
    if (file_provided) send_msg("Input Read Complete!");
    else
    {
        clear_screen();
        draw_string(10, 15, "No Input File!", BG_COLOUR);
        show_screen();
        _delay_ms(1000);
        Game_Over();
    }
}

char read_usb_command()
{
    if (usb_serial_available())
    {
        char usb_buffer[32];
        int c = usb_serial_getchar(); // read the first character
        switch (c)
        {
            case 'd':
            case 's':
            case 'a':            
            case 'w':            
            case 'f':            
            case 'i':
            case 'p':            
            case 'l':
                snprintf( usb_buffer, sizeof(usb_buffer), "received '%c'\r\n", c );
                usb_serial_send(usb_buffer);
                return c;

            default:
                return ' '; // as the function cannot terminate without a return char statment
        }
    }
    else return ' '; // same reason as default switch
}

/* 
Function to manage input from from both usb and joystick. Priority is USB over joystick.

Returns the input as a char.
*/
char input()
{
    if (game_lvl == 1) return check_joystick();
    else
    {
        char c = read_usb_command();
        if (c == ' ')
        {
            return check_joystick();
        }
        else return c;
    }    
}

void send_game_telemetery()
{
    // Time Stamp
    char buffer[100];
    snprintf(buffer, sizeof(buffer),  "%.2d:%.2d\r\n", game_min, (int)game_sec); //TODO: Check buffer print out and memory 
    send_msg_("Time stape: ", buffer);
    
    // Lvl
    snprintf(buffer, sizeof(buffer),  "%d\r\n", game_lvl); //TODO: Check buffer print out and memory 
    send_msg_("Level: ", buffer);

    // Score
    snprintf(buffer, sizeof(buffer),  "%d\r\n", SCORE); //TODO: Check buffer print out and memory 
    send_msg_("Score: ", buffer);

    // Active Fireworks 
    int fireworks_counter = 0;
    for (int i = 0; i < MAX_FIREWORKS; i++)
    {
        if (fireworks[i].visible) fireworks_counter++;
    } 
    snprintf(buffer, sizeof(buffer),  "%d\r\n", fireworks_counter); //TODO: Check buffer print out and memory 
    send_msg_("Fireworks On Screen: ", buffer);
    
    // Active Mousetraps
    snprintf(buffer, sizeof(buffer),  "%d\r\n", game_traps); //TODO: Check buffer print out and memory 
    send_msg_("Mousetraps On Screen: ", buffer);
    
    // Active Cheeses
    snprintf(buffer, sizeof(buffer),  "%d\r\n", game_cheeses); //TODO: Check buffer print out and memory 
    send_msg_("Cheeses On Screen: ", buffer);
    
    // CheesesEaten by jerry
    snprintf(buffer, sizeof(buffer),  "%d\r\n", game_cheeseEaten); //TODO: Check buffer print out and memory 
    send_msg_("Cheeses Consumed: ", buffer);
    
    // Lives of jerry //TODO: REMOVE
    snprintf(buffer, sizeof(buffer),  "%d\r\n", game_lives); //TODO: Check buffer print out and memory 
    send_msg_("Lives: ", buffer);
    
    // Supermode?
    send_msg_("Jerry_small Super-Mode on: ", (SuperMode == 1 ? "True\r\n" : "False\r\n"));
    
    // paused?
    send_msg_("Game Paused: ", (game_paused ? "True\r\n" : "False\r\n"));    
}

//TODO: Add description 
void fire()
{
    //setup the firework 
    for (int i = 0; i < MAX_FIREWORKS; i++)
    {
        if(!fireworks[i].visible)
        {
            //Fire one weapon at a time 
            setup_firework(i);
            fireworks[i].visible = true; // Make it visible on the screen
            break;
        }
    } 
}

/* reads the pixel in screen buffer at coordinates x,y.

Returns the value of the pixel as int. */
int read_pixel(int x, int y)
{
    	// Do nothing if requested pixel is out of bounds. 
	if ( x < 0 || y < 0 || x >= LCD_X || y >= LCD_Y ) {
		return 0;	}

	// Calculate the pixel position, within that LCD bank
	uint8_t bank = y >> 3;
	uint8_t pixel = y & 7;

    //read the value
    return BIT_IS_SET(screen_buffer[bank*LCD_X + x], pixel);
}

//TODO: Add Description
int isValidLocation2(uint8_t bitmap[], int x, int y, int width, int height)
{
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            if (BIT_IS_SET(bitmap[j], 7 - i))
            {
                if (BIT_IS_ON(screen_buffer[x + j], y + (7 - i))) return false;
               //if the bit is on in bitmap 
                // check if the bit is on in the screen buffer
                    //if yes than return false 
            }            
        }        
    }
    // else
    return true;
}

/* Performs the randomised movement of Tom. If Tom collides with walls or bondaries of the game a new randomised velocity is generated to continue moving. */
void randomised_movement()
{
    uint8_t bounced = false;
    // Predict the next step of the Tom
    int next_x = round(Tom.x + Tom.dx);
    int next_y = round(Tom.y + Tom.dy);

    //Check for collisions
    if(next_x == 1 || next_x == (GAMEPLAY_X - Tom_width + 1))// || !clear_of_wall_collisions_x(next_x, next_y)) //If on the left or the right border  or colliding with a vertical wall switch direction horizontally
    {
        initialise_tom_movement();  
        bounced = true;
    }
    if (next_y == (STATUS_BAR_HEIGHT) || next_y == (GAMEPLAY_Y - Tom_height + 1)) // || !clear_of_wall_collisions_y(next_x, next_y)) //if on the top or bottom border or colliding with a horizontal wall switch direction vertically
    {
        initialise_tom_movement();
        bounced = true;
    }

    if(!bounced)
    {
        Tom.x += Tom.dx * 0.8 * jerry_small_speed;
        Tom.y += Tom.dy * 0.8 * jerry_small_speed;
    }
}

/*pause_game() changes the state of the game to pause mode. */
void pause_game()
{
    if(!game_paused) {
        game_paused = true;
        game_pause_time = overflow_counter3_time; // store the number of seconds the game was paused at
    }
    else {
        game_paused = false;
        overflow_counter3_time = game_pause_time;
    }
}

/* 
Performs a distance method comparison to check if the point (x, y) provided is on the line x1,y1 x2,y2 draws.

Returns true if pixel is on the line.
*/ 
int pixel_on_line(int x, int y, int x1, int y1, int x2, int y2)
{
    double dist_1 = calcDistanceBetween2Points(x, y, x1, y1); // Calcualte the distance between the pixel and the x1 y1 corrdinate

    double dist_2 = calcDistanceBetween2Points(x, y, x2, y2); // do the same with x2 y2 coordiante

    double dist_3 = calcDistanceBetween2Points(x1, y1, x2, y2); // calc the comparison distance

    return (absolute(dist_3 - (dist_1 + dist_2)) == TOLERANCE);
}

/* 
Wrapper function for pixel_on_line() call it on each wall.

Returns true if pixel is on any wall
*/
int pixel_on_wall(int x, int y)
{
    for (int i = 0; i < MAX_WALLS; i++)
    {
        if (pixel_on_line(x, y, walls[i].x1, walls[i].y1, walls[i].x2, walls[i].y2)) { return true; }
    }
    return false;
}

/* 
Performs a wall collsion check on all walls on passed in coordinates.

Returns  true if the coordinate is found to be clear of all walls.
*/
int clear_of_wall_collisions(int x_check, int y_check)
{
    for (int i = 0; i < Jerry_width; i++)
    {
        for (int j = 0; j < Jerry_height; j++)
        {
            if (BIT_IS_SET(jerry[j], 7 - i)) // only check for collision if the bit is on
            {
                for (int w = 0; w < MAX_WALLS; w++)
                {

                    if ( pixel_on_line(x_check + i, y_check, walls[w].x1, walls[w].y1, walls[w].x2, walls[w].y2) )
                    {
                        send_msg("Wall collision!");
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

//TODO: ADD description
void reset_game()
{
    // TODO: add logic
    clear_screen();
    initialise_game();
    setup_players();
    initialise_game_objects();
    initialise_tom_movement();
    overflow_counter3_time = 0;
    overflow_coounter2_traps = 0;
    overflow_counter1_cheese = 0;
    overflow_counter2_milk_potion = 0;
    if (game_gameOver) game_gameOver = false; // Restart play
}

//TODO: ADD description
void show_customised_message()
{
    char buffer[50];
    // customise top of the message
    if (game_lives <= 0) draw_string(18, 5, " Game Over! ", BG_COLOUR); 
    else draw_string(18, 5, " You Won! ", BG_COLOUR);
    
    draw_formatted(23, 15, buffer, sizeof(buffer), FG_COLOUR, "Score: %d", SCORE);
    draw_string(9, 30, "SW3 to restart", FG_COLOUR);
}

//TODO: ADD description
void Game_Over()
{
    if(game_lvl < MAX_LVL && game_lives > 0)
    {
        send_msg("Leveling up!");
        game_lvl++;
        next_level();
        return; // Proceed to the next room
    }

    clear_screen();
    show_customised_message();
    show_screen();

    while (true)
    {
        char buffer[50];
        itoa(right_SW_pressed, buffer,10);
        char *msg = buffer;
        usb_serial_send(msg);

        if (right_SW_pressed)
        {
            send_msg("Rest Game!");
            reset_game();
            return;
        }
        _delay_ms(100);
    }    
}

/* Handle movement of both Tom and Jerry_small in the game based on input and other game paramets. */
void update_players()
{
    clear_bitmap((SuperMode ? jerry_large : jerry), Jerry.x, Jerry.y);

    //Update Jerry_small's coords
    switch (input())
    {
        case 'a':   
        if (Jerry.x > 2)
        { 
            double next_x = Jerry.x - jerry_small_speed;

            Jerry.x = (!SuperMode && clear_of_wall_collisions(round(next_x), round(Jerry.y)) ? next_x : (double)Jerry.x);
        }
        break;
        
        case 'w':
            if (Jerry.y > (STATUS_BAR_HEIGHT + 1)) 
            { 
                //Check before moving the next step  
                double next_y = Jerry.y - jerry_small_speed;

                Jerry.y = (!SuperMode && clear_of_wall_collisions(round(Jerry.x), round(next_y)) ? next_y : (double)Jerry.y);
            }
            break;
        
        case 's':
            if(Jerry.y < GAMEPLAY_Y - Jerry_height) 
             { 
                //Check before moving the next step  
                double next_y = Jerry.y + jerry_small_speed;

                Jerry.y = (!SuperMode && clear_of_wall_collisions(round(Jerry.x), round(next_y)) ? next_y : (double)Jerry.y);
            }
            break;
        
        case 'd':
            if(Jerry.x < (GAMEPLAY_X - Jerry_width))
            { 
                double next_x = Jerry.x + jerry_small_speed;

                Jerry.x = (!SuperMode && clear_of_wall_collisions(round(next_x), round(Jerry.y)) ? next_x : (double)Jerry.x);
            }
            break; 
        
        case 'f':
            if (game_cheeseEaten >= 3) fire();
            break;

        case 'p':
            pause_game();
            break;

        case 'i':
            send_game_telemetery();
            break;

        case ' ':
            break; 

        default:
            break;
    }

    draw_bitmap((SuperMode ? jerry_large : jerry), Jerry.x, Jerry.y);

    // update the bounding box
    Add_bounding_box(&Jerry.bounding_box, Jerry.y, Jerry.x, Jerry_height, Jerry_width);

    if (SuperMode)
    {
        if (dutyCycle > 254 || (dutyCycle < 254 && downwards == true) ) // Duty Cycle is currently up
        {
            dutyCycle -= 40;
            downwards = true; // Turn it down
        }

        if (dutyCycle < 0 || (dutyCycle < 254 && downwards == false) ) // Duty Cycle is currently down
        {
            dutyCycle += 40;
            downwards = false; // Turn it back up
        }
    }

    // ---------------------- TOM ---------------------------------
    if (!game_paused) // if not paused move Tom
    {
        //Update Tom's Coords
        clear_bitmap(tom, round(Tom.x), round(Tom.y));
        randomised_movement();
        draw_bitmap(tom, round(Tom.x), round(Tom.y));

        // store the bounding box properties
        Add_bounding_box(&Tom.bounding_box, Tom.y, Tom.x, Tom_height, Tom_width);
    }

    if (collided(&Jerry.bounding_box, &Tom.bounding_box)) 
    {
        if (!SuperMode) 
        {
            game_lives--;
            if (game_lives <= 0) Game_Over();
        }
        else
        {
            carryOver_score++;
        }
        
        // clear_bitmap(jerry, round(Jerry.x), round(Jerry.y));
        // clear_bitmap(tom, round(Tom.x), round(Tom.y));
        reset_players();
    }    
}

/* Handles moving of the fireworks and check for any collision with Tom. */
void update_fireworks()
{
    // check for weapon's collision with Tom
    for (int i = 0; i < MAX_FIREWORKS; i++)
    {
        if (fireworks[i].visible) //only check if they are active
        {
            if ((int)Tom.x == (int)fireworks[i].x && (int)Tom.y == (int)fireworks[i].y)
            {
                game_firework_hit++;
                fireworks[i].visible = false; // remove the pixel
                draw_pixel(round(fireworks[i].x), round(fireworks[i].y), BG_COLOUR); // remove firework
                clear_bitmap(tom, round(Tom.x), round(Tom.y));
                reset_player(&Tom, Tom_width, Tom_height);
                break; // No point checking others as Tom no longer is in the same posoition
            }
        }
    }

    if ( game_lives <= 0) game_gameOver = true;
    
    //update all the visible firework's coordinates 
    for (int i = 0; i < MAX_FIREWORKS; i++)
    {
        if (fireworks[i].visible) 
        {
            draw_pixel(round(fireworks[i].x), round(fireworks[i].y), BG_COLOUR); // remove the pixel
            move_weapon(fireworks[i].x, fireworks[i].y, i); // if they are on the screen than move them
            draw_pixel(round(fireworks[i].x), round(fireworks[i].y), FG_COLOUR);

            // if firework hits any walls or moves outside the gameplay area turn it off
            if ( pixel_on_wall(round(fireworks[i].x), round(fireworks[i].y)) || \
                (round(fireworks[i].x) < 1 || round(fireworks[i].x) > GAMEPLAY_X ) || \
                (round(fireworks[i].y) < STATUS_BAR_HEIGHT + 1 || round(fireworks[i].y) > GAMEPLAY_Y) )
            {
                fireworks[i].visible = false;
                draw_pixel(round(fireworks[i].x), round(fireworks[i].y), BG_COLOUR);
            } 
        }
    }
}

/* Handles initialisation of cheeses and collision of cheeses with Jerry. */
void update_cheese()
{
     //Draw cheese if the timer has expired and there are less than 5 cheeses on screen
    if( (overflow_counter1_cheese >= Cheese_timer_MAX ) && game_cheeses < 5 && !game_paused) //Drop cheese every 2 seconds
    {
        for (int i = 0; i < MAX_cheeses; i++)
        {
            if(!cheeses[i].visible) 
            {
                cheeses[i].visible = true;
                draw_bitmap(cheese, cheeses[i].x, cheeses[i].y);
                game_cheeses++;
                overflow_counter1_cheese = 0; // Reset the timer
                break;
            }
        }
    }

    //Check if the Jerry_small collides with any of the cheeses 
    for(int i = 0; i < MAX_cheeses; i++)
    {
        if (cheeses[i].visible && collided(&Jerry.bounding_box, &cheeses[i].bounding_box))
        {
            game_cheeseEaten++;
            game_cheeses--;
            
            clear_bitmap(cheese, cheeses[i].x, cheeses[i].y); // Remove it from the screen
            cheeses[i].visible = false; //Stop the cheese from apperaing again
            setup_cheese(i); //reinitialise the cheese for next round
            
            overflow_counter1_cheese = 0; // Reset the tiemr for 2 sec. 
            
            if (game_cheeseEaten > 4 && !Door.visible)
            {
                setup_door();
                Door.visible = true;
            }
            break;
        }
    }
}

/* Handles initialisation of traps and intraction of traps with Jerry. */
void update_traps()
{
     //Draw traps if the timer has expired and there are less than 5 traps on screen
    if( (overflow_coounter2_traps >= Trap_timer_MAX ) && game_traps < 5 && !game_paused) //Drop traps every 3 seconds
    {
        for (int i = 0; i < MAX_traps; i++)
        {
            if(!traps[i].visible) 
            {
                traps[i].visible = true;
                traps[i].x = round(Tom.x);
                traps[i].y = round(Tom.y);
                draw_bitmap(trap, traps[i].x, traps[i].y);
                game_traps++;

                // store the bounding box properties
                Add_bounding_box(&traps[i].bounding_box, traps[i].y, traps[i].x, trap_height, trap_width);

                overflow_coounter2_traps = 0; // Reset the timer
                break;
            }
        }
    }

    //Check if the Jerry_small collides with any of the traps
    if (!SuperMode)
    {
        for(int i = 0; i < MAX_traps; i++)
        {
            if (traps[i].visible && collided(&Jerry.bounding_box, &traps[i].bounding_box))
            {
                game_lives--;
                if (game_lives <= 0 ) Game_Over();
                game_traps--;
                clear_bitmap(trap, traps[i].x, traps[i].y);
                traps[i].visible = false; //Stop the cheese from apperaing again 
                overflow_coounter2_traps = 0; // Reset the tiemr for 2 sec. 
                break;
            }
        }
    }
}

//TODO: ADD description
void update_milk_potion()
{
    if (overflow_counter2_milk_potion >= Milk_potion_timer_MAX && !milk_potion.visible)
    {
        setup_milk_potion();
        milk_potion.visible = true;
    }

    //Check for collision with Jerry_small
    if (collided(&Jerry.bounding_box, &milk_potion.bounding_box)) 
    {
        SuperMode = true; //Enable SuperMode
        ticker = 0; // reset PWM

        // TODO: Change the bit mask to the greater one
        // jerry = jerry_large;
        // Jerry_width = 8; Jerry_height = 8;

        milk_potion.visible = false;
        clear_bitmap(potion, milk_potion.x, milk_potion.y);
        overflow_counter2_milk_potion = 0; // reset the milk potion 
        overflow_counter2_SuperJerry_small = 0; // reset the milk potion 
        send_msg("SuperMode");
    }
}

//TODO: ADD description
void update_door()
{
    if (Door.visible == true && collided(&Jerry.bounding_box, &Door.bounding_box))
    {
        Door.visible = false;
        send_msg("Game Over!");
        Game_Over();
    }
}

// ---------------- DRAW FUNCTIONS -------------------------
/* Draw the parssed bitmap at from coordinates (x, y) with provided color. */ 
void draw_bitmap_(uint8_t bitmap[], int x, int y, colour_t color)
{
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            if (BIT_VALUE(bitmap[j], 7 - i))
            {
                draw_pixel(x+i, y+j, color);
            }
        }
    }   
}

//TODO: Add description
void draw_walls(){
	//draw 4 lines
    for (int i = 0; i < MAX_WALLS; i++)
    {
        draw_line(round(walls[i].x1), round(walls[i].y1), round(walls[i].x2), round(walls[i].y2), FG_COLOUR);
    }   
}

/* Draw the players on screen. */
void draw_players()
{
    draw_bitmap(tom, round(Tom.x), round(Tom.y));
    draw_bitmap((SuperMode ? jerry_large : jerry), Jerry.x, Jerry.y);
}


/* 
Handle formatting and display of status bar components. 
Plus draw the bounding box for game_
*/
void draw_status_bar()
{
    char buffer[80];

    // Draw the game telemetry in format of L:1 J:5 S:0 T: 00:00
    draw_formatted(0,0, buffer, sizeof(buffer), BG_COLOUR, "L:%d", game_lvl, BG_COLOUR); // Dropping 'L:' due to space constraints 
    draw_bitmap(filler, 15, 0);
    draw_bitmap(filler, 17, 0);
    draw_formatted(29,0, buffer, sizeof(buffer), BG_COLOUR, "%d ", game_lives);
    draw_bitmap(heart, 19, 0);
    draw_bitmap(filler, 27, 0);
    draw_formatted(38, 0, buffer, sizeof(buffer), BG_COLOUR, "S:%d ", SCORE);
    draw_line(0, 9, GAMEPLAY_X, 9, FG_COLOUR);

    // Draw a bounding box for the game play area
    draw_line(0, STATUS_BAR_HEIGHT, 0, GAMEPLAY_Y, FG_COLOUR);
    draw_line(0, GAMEPLAY_Y, GAMEPLAY_X, GAMEPLAY_Y, FG_COLOUR);
    draw_line(GAMEPLAY_X, STATUS_BAR_HEIGHT, GAMEPLAY_X, GAMEPLAY_Y, FG_COLOUR);

    show_screen();
}

/* Draw parssed object on scree. */
void draw_objects(struct object objects[], uint8_t bitmap[], int MAX)
{
    for (int i = 0; i < MAX; i++)
    {
        if (objects[i].visible) draw_bitmap(bitmap, objects[i].x, objects[i].y);
    }
}

// Draw parssed object if its visible property is enabled.
void draw_object(struct object *object, uint8_t bitmap[])
{
    if (object->visible == 1) draw_bitmap(bitmap, object->x, object->y);
}

/* Draws all the visible fireworks. */
void draw_fireworks()
{
    for (int i = 0; i < MAX_FIREWORKS; i++)
    {
        if (fireworks[i].visible) draw_pixel(round(fireworks[i].x), round(fireworks[i].y), FG_COLOUR);
    }
}

/* Caller function to handle drawing of all game elements. */
void draw_all()
{
    draw_walls();
    draw_cheeses();
    draw_traps();
    draw_milk_potion();
    draw_door();
}

/* updates the game to advance on to the next level in the game. */
void next_level()
{
    // clear the room 
    for (int i = 0; i < walls_index; i++)
    {
        walls[i].x1 = 0;
        walls[i].x2 = 0;
        walls[i].y1 = 0;
        walls[i].y2 = 0;
    }    
    walls_index = 0;
    clear_bitmap((SuperMode ? jerry_large : jerry), Jerry.x, Jerry.y);
    Jerry.x = 0; Jerry.y = 0;
    clear_bitmap(tom, Tom.x, Tom.y);
    Tom.x = 30; Tom.y = 30;
    initialise_game_objects(); //clear objects & reinitialise them

    // Refresh game parameters
    carryOver_score = SCORE;
    game_cheeseEaten = 0;
    game_firework_hit = 0;
    game_paused = false;
    game_gameOver = false;
    game_cheeses = 0;
    game_traps = 0;

    send_msg("Reading Inputfile ...");
    read_usb_inputfile();

    initialise_tom_movement();
    draw_players();
    // reset timers
    overflow_counter1_cheese = 0; overflow_coounter2_traps = 0; overflow_counter2_milk_potion = 0;
}

/* Move the walls in the game. */
void update_walls()
{
    // send_double((double)walls_index);
    for (int i = 0; i < (walls_index + 1); i++)
    {
        draw_line(round(walls[i].x1), round(walls[i].y1), round(walls[i].x2), round(walls[i].y2), BG_COLOUR);

        double x1, x2, y1, y2;

        send_msg("wall"); send_double(i);
        send_msg("dx update_wall"); send_double(walls[i].dx);
    
        x1 = WRAP(walls[i].x1 + walls[i].dx * wall_speed, GAMEPLAY_X - 1 + absolute(walls[i].x1 - walls[i].x2));
        x2 = WRAP(walls[i].x2 + walls[i].dx * wall_speed, GAMEPLAY_X - 1 + absolute(walls[i].x1 - walls[i].x2));

        walls[i].x1 =  x1;
        walls[i].x2 = x2;

        y1 = WRAP(walls[i].y1 + walls[i].dy * wall_speed, GAMEPLAY_Y + absolute(walls[i].y1 - walls[i].y2));
        y2 = WRAP(walls[i].y2 + walls[i].dy * wall_speed, GAMEPLAY_Y + absolute(walls[i].y1 - walls[i].y2));

        walls[i].y1 = y1;
        walls[i].y2 = y2;

        draw_line(round(walls[i].x1), round(walls[i].y1), round(walls[i].x2), round(walls[i].y2), FG_COLOUR);
    }
}
//**********************************************************

//TODO: add description
void process()
{
    // runs the processes
    
    draw_status_bar();
    read_potentiometers();
    if (right_SW_pressed) pause_game();
    if(!game_paused) 
    {
        update_time();
    }

    draw_all();

    // update_walls();
    update_players();
    update_fireworks();
    update_cheese();
    update_traps();
    update_milk_potion();
    update_door();
}

/* Setup the attributes and screen for the game play. It initialises the LCD screen, handles switch and timer setup and displays welcome screen.  */
void setup()
{
    //Setup Hardware components
    set_clock_speed(CPU_8MHz);
    usb_init(); //Ensure USB connection 
    adc_init();
    lcd_init(LCD_DEFAULT_CONTRAST);
    clear_screen();

    while ( !usb_configured() ) {}; // Hold game procedures until a usb connection is configured

    //Setup the switches and LED's
    CLEAR_BIT(DDRF, 6); //setup SW 1 
    CLEAR_BIT(DDRF, 5); // setup SW 2
    setup_LED(0);
    setup_LED(1);

    setup_timer0();
    setup_timer3();
    sei(); // Enable gloabal interrupts

    //TODO: REMOVE
    char *msg1 = "welcome\n";
    usb_serial_send(msg1);

    // Show welcome screen 
    show_welcome_screen();
    initialise_game();

    srand(rnd);
    initialise_game_objects();

    // Setup the rest of the game
    // setup_players();
    // setup_walls();

    read_usb_inputfile(); // Enable this to test usb connection
    initialise_tom_movement();
    draw_players();    

    draw_status_bar();

    // Setup remaining timers
    setup_timer1();
    // setup_timer3();

    setup_joystick();

    sei();
    
}



// ---------------------------------------------------------

//TODO: add description
int main() 
{
    setup(); // Setup the game
    
    while(1)
    {
        process();
        _delay_ms(50);
    }
    
    return 0;
} 