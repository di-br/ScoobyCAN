/*
 * Original work copyright 2013 Fabio Baltieri <fabio.baltieri@gmail.com>
 * Modified work copyright 2015-2017 di-br
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <curses.h>
#include <endian.h>
// try and get timestamps
#include <time.h>
#include <sys/time.h>

static void unknown_frame(int);
static void process_one(struct can_frame *frm);
static int ncurses_init(void);
static int paint_empty_scr(void);
static int tpm_check(int *tpm_flag);
static int mem_init(void);
static int net_init(char *ifname);
static void receive_one(void);
int main(int argc, char **argv);

#define __packed __attribute__((packed))
#define rbi(b,n) ((b) & (1<<(n)))           /* Read bit number n in byte b    */

// set up a fudge factor to guess better fuelconsumption
#define FUELFUDGE 64.

// minimal size for ncurses window
#define LINES 25
#define WIDTH 90
// where we put what, coordinates for ncurses window
#define STEER_LINE 2
#define AVG_SPEED_LINE 5
#define IND_SPEED_LINE 7
#define TEMP_LINE 11
#define FUEL_LINE 13
#define ENGINE_LINE 14
#define SWITCHES_LINE -4
#define LEFT_WHL 25
#define MID_WHL 42
#define RIGHT_WHL 59
#define STEER_COL 52
#define COOL_COL 50
#define RPM_COL 22
#define ACCEL_COL 50
// COLORS
#define HEADING 1
#define WARN 2
#define HIL 3

// limit on the unknown frames, for non-extended CAN this is < 0x800
#define UNKNOWN_COUNT 1024
static int unknown[UNKNOWN_COUNT];

int row, col; // global size of our window
int display;  // this controlls how often we update the screen or output data
int tpm_flag[4]; // this will hold data on TPM module

// ENUM the frame IDs we know are present or know how to interpret
enum frame_ids {
	SUB_STEERING_SENSOR = 0x002,
	SUB_VCDS_Y = 0x70,
	SUB_VCDS_X = 0x80,
	SUB_ECU_410 = 0x410,
	SUB_ECU_411 = 0x411,
	SUB_VCDS_TORQ = 0x501,
	SUB_VCDS_STEERING_SENSOR = 0x511,
	SUB_VCDS_SPEED = 0x512,
	SUB_VCDS_SPEEDS = 0x513,
	SUB_BIU_TEMP = 0x514,
	SUB_ECU_600 = 0x600,
	SUB_BIU_620 = 0x620,
	FRAME_COUNT = 12
};

// index switches we identified
enum switch_data {
   BREAK_SW,
   CLUTCH_SW,
   DOOR_SW,
   SWITCH_COUNT
};
bool switches[SWITCH_COUNT];

// index floats we want to use
enum float_data {
   ACCEL,           // col 7
   A_X,             // col 8
   A_Y,             // col 9
   SPEED,           // col 10
   SPEED_F_L,       // col 11
   SPEED_F_R,       // col 12
   SPEED_R_L,       // col 13
   SPEED_R_R,       // col 14
   TRANS_TORQ,      // col 15
   ENGINE_TORQ,     // col 16
   TORQ_LOSS,       // col 17
   FLOAT_COUNT
};
float float_mem[FLOAT_COUNT];
float maxf, minf, minax, maxax, minay, maxay;

// index ints we want to use
enum int_data {
   STEER_VAL,       // col 2
   STEER_ANGLE,     // col 3
   RPM,             // col 4
   FUEL,            // col 5
   GEAR,            // col 6
   INT_COUNT
};
int32_t int_mem[INT_COUNT];

// a union to access the frame content, same memory, different interpretation
union u_frames {
   struct __packed { // 0x002
      int16_t angle;
      uint8_t byte2;
      uint8_t byte3;
      uint8_t byte4;
      uint8_t byte5;
      uint8_t byte6;
      uint8_t byte7;
   } steering_sensor;
   struct __packed { // 0x070
      uint16_t yaw_rate;
      uint8_t byte2;
      uint8_t byte3;
      uint16_t y_accel;
      uint8_t byte6;
      uint8_t byte7;
   } vcds_y;
   struct __packed { // 0x080
      uint16_t yaw_accel;
      uint8_t byte2;
      uint8_t byte3;
      uint16_t x_accel;
      uint8_t byte6;
      uint8_t byte7;
   } vcds_x;
   struct __packed { // 0x410
      uint8_t byte0;
      uint8_t transtorq;
      uint8_t engtorq;
      uint8_t torqloss;
      uint8_t accel;
      uint16_t rpm;
      uint8_t byte7;
   } ecu_410;
   struct __packed { // 0x411
      uint8_t byte0;
      uint16_t le;
      uint8_t byte3;
      uint8_t gear;
      uint8_t cruisespeed;
      uint8_t byte6;
      uint8_t byte7;
   } ecu_411;
   struct __packed { // 0x501
      uint8_t byte0;
      uint8_t byte1;
      uint8_t torqreduction;
      uint8_t torqallow;
      uint8_t torqdown;
      uint8_t _counter;
      uint8_t byte6;
      uint8_t byte7;
   } vcds_torq;
   struct __packed { // 0x511
      int16_t angle;
      uint8_t byte2;
      uint8_t byte3;
      uint8_t byte4;
      uint8_t byte5;
      uint8_t byte6;
      uint8_t byte7;
   } vcds_steering_sensor;
   struct __packed { // 0x512
      uint8_t byte0;
      uint8_t byte1;
      int16_t speed;
      uint8_t byte5;
      uint8_t counter;
      uint16_t fcode;
   } vcds_speed;
   struct __packed { // 0x513
      int16_t frle;
      int16_t frri;
      int16_t rele;
      int16_t reri;
   } vcds_speeds;
   struct __packed { // 0x514
      uint8_t byte0;
      uint8_t byte1;
      int16_t temp;
      uint8_t leftlever;
      uint8_t fuel;
      uint8_t counter;
   } biu_temp;
   struct __packed { // 0x600
      uint8_t dpf_bits;
      uint16_t fuel;
      uint8_t coolant;
      uint8_t counter;
      uint8_t byte5;
      uint8_t clutch_bits;
      uint8_t byte7;
   } ecu_600;
   struct __packed {
      uint8_t byte0;
      uint8_t byte1;
      uint8_t byte2;
      uint8_t byte3;
      uint8_t byte4;
      uint8_t byte5;
      uint8_t byte6;
      uint8_t byte7;
   } biu_620;
   struct __packed {
      uint8_t byte0;
      uint8_t byte1;
      uint8_t byte2;
      uint8_t byte3;
      uint8_t byte4;
      uint8_t byte5;
      uint8_t byte6;
      uint8_t byte7;
   } raw;
};

static int can_socket;
struct timeval tv;

// functions start here
//
// deal with unknown frames
static void unknown_frame(int id)
{
	int i;

	for (i = 0; i < UNKNOWN_COUNT; i++)
		if (unknown[i] == 0 || unknown[i] == id)
			break;
	if (i == UNKNOWN_COUNT)
		return;

	unknown[i] = id;

#ifdef NCURS
	move(row - 3, 1);
	clrtoeol();
	mvprintw(row - 3, 1, "unknown frames:");
	for (i = 0; i < UNKNOWN_COUNT; i++) {
		if (unknown[i] == 0)
			break;
		printw(" %02x", unknown[i]);
	}
	printw(" (%d)", i);
#endif
}

// process single CAN frame
static void process_one(struct can_frame *frm)
{
        int i;
	union u_frames *msg;

	msg = (union u_frames *)frm->data;

	switch (frm->can_id) {
        case SUB_STEERING_SENSOR:
	   int_mem[STEER_VAL] = (int32_t) msg->steering_sensor.angle;
#ifdef NCURS
		mvprintw(STEER_LINE, STEER_COL, "%7d",
		      int_mem[STEER_VAL]);
#endif
		//refresh();
		break;
        case SUB_VCDS_Y:
		float_mem[A_Y] = (float) msg->vcds_y.y_accel*0.00012742 - 4.1768;
#ifdef NCURS
		mvprintw(18, 1, "yaw rate  %7.3f deg/s     y_accel %7.3f g",
		      (msg->vcds_y.yaw_rate*0.005 - 163.84),
		      float_mem[A_Y]);

		if (float_mem[A_Y] < minay)
		   minay = float_mem[A_Y];
		if (float_mem[A_Y] > maxay)
		   maxay = float_mem[A_Y];
		mvprintw(FUEL_LINE+9, RPM_COL+21, "rig %7.4f y_accel",
		                maxay);
		mvprintw(FUEL_LINE+8, RPM_COL+21, "lef %7.4f y_accel",
		                minay);
#endif
		break;
        case SUB_VCDS_X:
		float_mem[A_X] = (float) msg->vcds_x.x_accel*0.00012742 - 4.1768;
#ifdef NCURS
		mvprintw(19, 1, "yaw accel %7.3f deg/s^2   x_accel %7.3f g",
		      (msg->vcds_x.yaw_accel*0.125 - 4096),
		      float_mem[A_X]);

		if (float_mem[A_X] < minax)
		   minax = float_mem[A_X];
		if (float_mem[A_X] > maxax)
		   maxax = float_mem[A_X];
		mvprintw(FUEL_LINE+9, RPM_COL+45, "dec %7.4f x_accel",
		                maxax);
		mvprintw(FUEL_LINE+8, RPM_COL+45, "acc %7.4f x_accel",
		                minax);
#endif
		break;
	case SUB_ECU_410:
		int_mem[RPM] = (int32_t) msg->ecu_410.rpm;
		float_mem[ACCEL] = (float) msg->ecu_410.accel*100./255.;
		float_mem[TRANS_TORQ] = (float) msg->ecu_410.transtorq * 1.6;
		float_mem[ENGINE_TORQ] = (float) msg->ecu_410.engtorq * 1.6;
		float_mem[TORQ_LOSS] = (float) msg->ecu_410.torqloss * 1.6;
#ifdef NCURS
		mvprintw(ENGINE_LINE, RPM_COL, "%5d rpm",
		      int_mem[RPM]);
		mvprintw(ENGINE_LINE, ACCEL_COL, "%6.2f %",
		      float_mem[ACCEL]);
		mvprintw(ENGINE_LINE+1, RPM_COL, "%5.1f Nm %5.1f Nm %5.1f Nm   %5.1f",
		      msg->ecu_410.transtorq * 1.6,
		      msg->ecu_410.engtorq * 1.6,
		      msg->ecu_410.torqloss * 1.6,
		      (msg->ecu_410.transtorq-msg->ecu_410.engtorq) * 1.6);
#endif
	        break;
	case SUB_ECU_411:
		int_mem[GEAR] = (int32_t)msg->ecu_411.gear;
#ifdef NCURS
		mvprintw(AVG_SPEED_LINE, MID_WHL, "gear: %1d",
		      int_mem[GEAR]);
#endif
		if(rbi(msg->ecu_411.byte6,4) ^ switches[BREAK_SW])
		{
		   switches[BREAK_SW] ^= 1;
#ifdef NCURS
		   if (switches[BREAK_SW])
		   {
		      attron(A_BOLD | COLOR_PAIR(WARN));
		      mvprintw(row+SWITCHES_LINE, 3, "! BREAK !");
		      attroff(A_BOLD | COLOR_PAIR(WARN));
		   }
		   else
		      mvprintw(row+SWITCHES_LINE, 3, "         ");
#endif
		}
	        break;
	case SUB_VCDS_TORQ:
	        break;
        case SUB_VCDS_STEERING_SENSOR:
		int_mem[STEER_ANGLE] = (int32_t) msg->vcds_steering_sensor.angle;
#ifdef NCURS
		mvprintw(STEER_LINE+1, STEER_COL, "%7d DEG",
		      int_mem[STEER_ANGLE]);
#endif
		break;
        case SUB_VCDS_SPEED:
		float_mem[SPEED] = (float) msg->vcds_speed.speed * 0.05625;
#ifdef NCURS
		mvprintw(AVG_SPEED_LINE, LEFT_WHL, "%5.2f km/h",
		      float_mem[SPEED]);
		mvprintw(AVG_SPEED_LINE, col-5, "%5d",
		      (msg->vcds_speed.counter));
#endif
		break;
        case SUB_VCDS_SPEEDS:
		float_mem[SPEED_F_L] = (float) msg->vcds_speeds.frle * 0.05625;
		float_mem[SPEED_F_R] = (float) msg->vcds_speeds.frri * 0.05625;
		float_mem[SPEED_R_L] = (float) msg->vcds_speeds.rele * 0.05625;
		float_mem[SPEED_R_R] = (float) msg->vcds_speeds.reri * 0.05625;
#ifdef NCURS
		mvprintw(IND_SPEED_LINE, LEFT_WHL, "%5.2f km/h",
		                float_mem[SPEED_F_L]);
		mvprintw(IND_SPEED_LINE, RIGHT_WHL, "%5.2f km/h",
		                float_mem[SPEED_F_R]);
		mvprintw(IND_SPEED_LINE, MID_WHL, "%5.2f",
		                ((float_mem[SPEED_F_R]-float_mem[SPEED_F_L]) * 0.05625));
		mvprintw(IND_SPEED_LINE+1, LEFT_WHL, "%5.2f",
		                ((float_mem[SPEED_F_L]-float_mem[SPEED_R_L]) * 0.05625));
		mvprintw(IND_SPEED_LINE+1, RIGHT_WHL, "%5.2f",
		                ((float_mem[SPEED_F_R]-float_mem[SPEED_R_R]) * 0.05625));
		mvprintw(IND_SPEED_LINE+2, LEFT_WHL, "%5.2f km/h",
		                float_mem[SPEED_R_L]);
		mvprintw(IND_SPEED_LINE+2, RIGHT_WHL, "%5.2f km/h",
		                float_mem[SPEED_R_R]);
		mvprintw(IND_SPEED_LINE+2, MID_WHL, "%5.2f",
		                ((float_mem[SPEED_R_R]-float_mem[SPEED_R_L]) * 0.05625));

		// not check tire preassures
		tpm_check(tpm_flag);

#endif
		break;
        case SUB_BIU_TEMP:
#ifdef NCURS
		mvprintw(TEMP_LINE, RPM_COL, "%5.1f degC",
		                (msg->biu_temp.temp/2. - 40));
		mvprintw(TEMP_LINE, col-5, "%5d",
		                (msg->biu_temp.counter));
#endif
		break;
	case SUB_ECU_600:
		int_mem[FUEL] = (int32_t) msg->ecu_600.fuel;

		if (msg->ecu_600.fuel/FUELFUDGE < minf)
		   minf = msg->ecu_600.fuel/FUELFUDGE;
		if (msg->ecu_600.fuel/FUELFUDGE > maxf)
		   maxf = msg->ecu_600.fuel/FUELFUDGE;

		// compute l/h
		float lphr;
		// each rev sees two injections
		lphr = 2 * int_mem[FUEL]; // mm^3
		// each minute has rpm revs
		lphr *= int_mem[RPM];
		// now include mm^3 -> l factor and 60mins
		lphr *= 1.e-6;
		lphr *= 60;
		// add fudge factor...
		lphr /= FUELFUDGE;
		// compute l/1ookm
		float lph;
		// start w/ liters per hour
		lph = lphr;
		// normalise km/h to achieve 100km
		lph /= float_mem[SPEED_F_L];
		lph *= 100;

		if(rbi(msg->ecu_600.clutch_bits,2) ^ switches[CLUTCH_SW])
		{
		   switches[CLUTCH_SW] ^= 1;
#ifdef NCURS
		   if (switches[CLUTCH_SW])
		      mvprintw(row+SWITCHES_LINE, 13, "CLUTCH");
		   else
		      mvprintw(row+SWITCHES_LINE, 13, "      ");
#endif
		}
#ifdef NCURS
		mvprintw(FUEL_LINE, RPM_COL, "%5.2f mm3/s",
		                (int_mem[FUEL]/FUELFUDGE));
		mvprintw(FUEL_LINE+9, RPM_COL, "max %5.2f mm3/s",
		                maxf);
		mvprintw(FUEL_LINE+8, RPM_COL, "min %5.2f mm3/s",
		                minf);
		attron(COLOR_PAIR(HIL));
		mvprintw(FUEL_LINE+1, 2, "%6.1f l/1oo km",
		                lph);
		mvprintw(FUEL_LINE+2, 2, "%6.1f l/h",
		                lphr);
		attroff(COLOR_PAIR(HIL));
		mvprintw(FUEL_LINE, COOL_COL, "%5d degC",
		                (msg->ecu_600.coolant)-40);
		mvprintw(FUEL_LINE, col-5, "%5d",
		                (msg->ecu_600.counter));
#endif
	        break;
	case SUB_BIU_620:
		if( (rbi(msg->biu_620.byte0,5) ^ switches[DOOR_SW]))// || (rbi(msg->biu_620.byte2,1) ^ switches.door_sw))
		{
		   switches[DOOR_SW] ^= 1;
#ifdef NCURS
		   if (switches[DOOR_SW])
		   {
		      attron(A_BOLD | COLOR_PAIR(WARN));
		      mvprintw(row+SWITCHES_LINE, 23, " DOOR OPEN ");
		      attroff(A_BOLD | COLOR_PAIR(WARN));
		   }
		   else
		      mvprintw(row+SWITCHES_LINE, 23, "           ");
#endif
		}
	        break;
	default:
		unknown_frame(frm->can_id);
	}

	display += 1;
	if (display%5 == 0)
	{
	   gettimeofday(&tv, NULL);
#ifdef NCURS
	   mvprintw(row - 1, 1, "values for file [%010ld.%06ld]:",tv.tv_sec, tv.tv_usec);
	   for (i = 0; i < INT_COUNT; i++) {
	      printw(" %5d", int_mem[i]);
	   }
	   for (i = 0; i < FLOAT_COUNT; i++) {
	      printw(" %7.2f", float_mem[i]);
	   }
	   refresh();
#else
	   printf("%010ld.%06ld ",tv.tv_sec, tv.tv_usec);
	   for (i = 0; i < INT_COUNT; i++) {
	      printf(" %5d", int_mem[i]);
	   }
	   for (i = 0; i < FLOAT_COUNT; i++) {
	      printf(" %7.2f", float_mem[i]);
	   }
	   for (i = 0; i < SWITCH_COUNT; i++) {
	      printf(" %1d", switches[i]);
	   }
	   printf("\n");
#endif
	   display = 0;
	}

}

// init ncurses
static int ncurses_init(void)
{
   // init ncurses
   initscr();
   //pass C- on, keep the rest
   cbreak(); //raw();
   // do sth to F-keys
   keypad(stdscr, TRUE);
   // turn of echoing to screen
   noecho();

   if(has_colors() == FALSE)
   {
      endwin();
      printf("Your terminal does not support colour -- we want colour...\n");
      return 1;
   }
   // enable colour
   start_color();
   init_pair(HEADING, COLOR_BLACK, COLOR_CYAN);
   init_pair(WARN, COLOR_RED, COLOR_BLACK);
   init_pair(HIL, COLOR_GREEN, COLOR_BLACK);

   // get screen dimensions
   getmaxyx(stdscr,row,col);
   if (row < LINES || col < WIDTH)
   {
      endwin();
      printf("Sorry, your terminal is too small to fit all data.\nCurrent size is %2dx%2d\nMinimum size is %2dx%2d\n",row,col,LINES,WIDTH);
      return 1;
   }

   // paint initial screen w/o values
   paint_empty_scr();

   return 0;
}

// init ncurses screen and fill in some text
static int paint_empty_scr(void)
{
   int i = 1;
   for(i=1; i<=row; i++)
   {
      move(i, 1);
      clrtoeol();
   }
   //attron(A_BOLD | A_REVERSE | COLOR_PAIR(2));
   mvprintw(0, 0, " SCOOBY-DOO CAN VIEWER                                ");
   //attroff(A_BOLD | A_REVERSE | COLOR_PAIR(2));
   mvchgat(0, 0, -1, A_BOLD, 1, NULL);
   mvprintw(STEER_LINE, 1, "steering value:");
   mvprintw(STEER_LINE+1, 1, "steering angle degrees (LEFT -angle..angle RIGHT):");
   mvprintw(AVG_SPEED_LINE, 1, "avg front wheel speed:");
   mvprintw(AVG_SPEED_LINE, col-5-7, "msg cnt");
   mvprintw(IND_SPEED_LINE, 1, "front wheel speeds:");
   mvprintw(IND_SPEED_LINE+2, 1, "rear wheel speeds:");
   mvprintw(TEMP_LINE, 1, "ambient temperature:      degC");
   mvprintw(TEMP_LINE, col-5-7, "msg cnt");
   mvprintw(FUEL_LINE, 1, "fuel consumption:");
   mvprintw(FUEL_LINE, COOL_COL-13, "coolant temp:");
   mvprintw(FUEL_LINE, col-5-7, "msg cnt");
   mvprintw(ENGINE_LINE, ACCEL_COL-12, "accel ped:");
   mvprintw(ENGINE_LINE+2, 1, "Torques");
   mvprintw(ENGINE_LINE+2, RPM_COL, "  Transm   Engine     Loss");

   refresh;
   return 0;
}

// TPM - tire pressure module :-)
static int tpm_check(int *tpm_flag)
{
   enum pos {
      FRONT_WHLS,
      REAR_WHLS,
      LEFT_WHLS,
      RIGHT_WHLS,
      POS_COUNT
   };
   float diff[POS_COUNT], avg[POS_COUNT], rel[POS_COUNT];
   int angle = int_mem[STEER_ANGLE]*int_mem[STEER_ANGLE];

   // are we cornering?
   if (angle > TPM_STEER_LIMIT*TPM_STEER_LIMIT)
      return 0;

   // start by checking all wheel speeds against each other
   diff[FRONT_WHLS] = float_mem[SPEED_F_L] - float_mem[SPEED_F_R];
   diff[REAR_WHLS]  = float_mem[SPEED_R_L] - float_mem[SPEED_R_R];
   diff[LEFT_WHLS]  = float_mem[SPEED_F_L] - float_mem[SPEED_R_L];
   diff[RIGHT_WHLS] = float_mem[SPEED_F_R] - float_mem[SPEED_R_R];
   // look at average speeds for rel difference 
   avg[FRONT_WHLS] = (float_mem[SPEED_F_L] + float_mem[SPEED_F_R])/2.;
   avg[REAR_WHLS]  = (float_mem[SPEED_R_L] + float_mem[SPEED_R_R])/2.;
   avg[LEFT_WHLS]  = (float_mem[SPEED_F_L] + float_mem[SPEED_R_L])/2.;
   avg[RIGHT_WHLS] = (float_mem[SPEED_F_R] + float_mem[SPEED_R_R])/2.;
   // relative differences
   rel[FRONT_WHLS] = diff[FRONT_WHLS] / avg[FRONT_WHLS];
   rel[REAR_WHLS]  = diff[REAR_WHLS]  / avg[REAR_WHLS];
   rel[LEFT_WHLS]  = diff[LEFT_WHLS]  / avg[LEFT_WHLS];
   rel[RIGHT_WHLS] = diff[RIGHT_WHLS] / avg[RIGHT_WHLS];

   // clear text - in case there is one
   mvprintw(row+SWITCHES_LINE-4, 10, "                                    %5d ",tpm_flag[0]);
   mvprintw(row+SWITCHES_LINE-3, 10, "                                    %5d ",tpm_flag[1]);
   mvprintw(row+SWITCHES_LINE-2, 10, "                                    %5d ",tpm_flag[2]);
   mvprintw(row+SWITCHES_LINE-1, 10, "                                    %5d ",tpm_flag[3]);

   // identify fast one (this requires two above limit)
   // front left
   if ( rel[FRONT_WHLS] > 0.02 && rel[LEFT_WHLS] > 0.02 ) 
      tpm_flag[0]+=1; // it's faster
   else
      if (tpm_flag[0] < TPM_COUNT_LIMIT)  // we still think everything is ok, so reduce warn level
	 tpm_flag[0]-=1;
      else
	 if (tpm_flag[0] < 10*TPM_COUNT_LIMIT)  // we have issued warning, so stay alert for longer
	    tpm_flag[0]-=1;
   if (tpm_flag[0] > TPM_COUNT_LIMIT)
   {
      attron(A_BOLD | COLOR_PAIR(WARN) | A_REVERSE);
      mvprintw(row+SWITCHES_LINE-4, 10, "CHECK PREASSURE OF FRONT LEFT WHEEL!");
      attroff(A_BOLD | COLOR_PAIR(WARN) | A_REVERSE);
   }
   if (tpm_flag[0] < 0)
      tpm_flag[0] = 0;

   // front right
   if ( rel[FRONT_WHLS] < -0.02 && rel[RIGHT_WHLS] > 0.02 ) 
      tpm_flag[1]+=1; // it's faster
   else
      if (tpm_flag[1] < TPM_COUNT_LIMIT)  // we still think everything is ok, so reduce warn level
	 tpm_flag[1]-=1;
      else
	 if (tpm_flag[1] < 10*TPM_COUNT_LIMIT)  // we have issued warning, so stay alert for longer
	    tpm_flag[1]-=1;
   if (tpm_flag[1] > TPM_COUNT_LIMIT)
   {
      attron(A_BOLD | COLOR_PAIR(WARN) | A_REVERSE);
      mvprintw(row+SWITCHES_LINE-3, 10, "CHECK PREASSURE OF FRONT RIGHT WHEEL!");
      attroff(A_BOLD | COLOR_PAIR(WARN) | A_REVERSE);
   }
   if (tpm_flag[1] < 0)
      tpm_flag[1] = 0;

   // rear left
   if ( rel[REAR_WHLS] > 0.02 && rel[LEFT_WHLS] < -0.02 ) 
      tpm_flag[2]+=1; // it's faster
   else
      if (tpm_flag[2] < TPM_COUNT_LIMIT)  // we still think everything is ok, so reduce warn level
	 tpm_flag[2]-=1;
      else
	 if (tpm_flag[2] < 10*TPM_COUNT_LIMIT)  // we have issued warning, so stay alert for longer
	    tpm_flag[2]-=1;
   if (tpm_flag[2] > TPM_COUNT_LIMIT)
   {
      attron(A_BOLD | COLOR_PAIR(WARN) | A_REVERSE);
      mvprintw(row+SWITCHES_LINE-2, 10, "CHECK PREASSURE OF REAR LEFT WHEEL!");
      attroff(A_BOLD | COLOR_PAIR(WARN) | A_REVERSE);
   }
   if (tpm_flag[2] < 0)
      tpm_flag[2] = 0;

   // rear right
   if ( rel[REAR_WHLS] < -0.02 && rel[LEFT_WHLS] < -0.02 ) 
      tpm_flag[3]+=1; // it's faster
   else
      if (tpm_flag[3] < TPM_COUNT_LIMIT)  // we still think everything is ok, so reduce warn level
	 tpm_flag[3]-=1;
      else
	 if (tpm_flag[3] < 10*TPM_COUNT_LIMIT)  // we have issued warning, so stay alert for longer
	    tpm_flag[3]-=1;
   if (tpm_flag[3] > TPM_COUNT_LIMIT)
   {
      attron(A_BOLD | COLOR_PAIR(WARN) | A_REVERSE);
      mvprintw(row+SWITCHES_LINE-1, 10, "CHECK PREASSURE OF REAR RIGHT WHEEL!");
      attroff(A_BOLD | COLOR_PAIR(WARN) | A_REVERSE);
   }
   if (tpm_flag[3] < 0)
      tpm_flag[3] = 0;

   return 0;
}

// init our global data structures
static int mem_init(void)
{
   int i;

   maxf = 0.0;
   minf = 1000000.0;
   maxax = 0.0;
   minax = 1000000.0;
   maxay = 0.0;
   minay = 1000000.0;
   display = 0;
   tpm_flag[0] = 0;
   tpm_flag[1] = 0;
   tpm_flag[2] = 0;
   tpm_flag[3] = 0;

   // init switches
   switches[BREAK_SW] = 1;
   switches[CLUTCH_SW] = 1;
   switches[DOOR_SW] =1;

   // init values
   for (i = 0; i < INT_COUNT; i++) {
      int_mem[i] = 0;
   }
   // init float values
   for (i = 0; i < FLOAT_COUNT; i++) {
      float_mem[i] = 0.0;
   }

   return 0;
}

static int net_init(char *ifname)
{
   int recv_own_msgs;
   struct sockaddr_can addr;
   struct ifreq ifr;

   can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
   if (can_socket < 0) {
      perror("socket");
      exit(1);
   }

   memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
   strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
   if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
      perror("SIOCGIFINDEX");
      exit(1);
   }

   memset(&addr, 0, sizeof(addr));
   addr.can_family = AF_CAN;
   addr.can_ifindex = ifr.ifr_ifindex;
   if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      perror("bind");
      return 1;
   }

   recv_own_msgs = 0; /* 0 = disabled (default), 1 = enabled */
   setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
	 &recv_own_msgs, sizeof(recv_own_msgs));

   return 0;
}

static void receive_one(void)
{
   struct can_frame frm;
   struct sockaddr_can addr;
   int ret;
   socklen_t len;

   ret = recvfrom(can_socket, &frm, sizeof(struct can_frame), 0,
	 (struct sockaddr *)&addr, &len);
   if (ret < 0) {
      perror("recvfrom");
      exit(1);
   }

   process_one(&frm);
}

int main(int argc, char **argv)
{
   printf("known frame IDs: %d\n",FRAME_COUNT);
   printf("monitored floats: %d\n",FLOAT_COUNT);
   printf("monitored ints: %d\n\n\n",INT_COUNT);
   gettimeofday(&tv, NULL);
   printf("current timestamp [%010ld.%06ld]\n\n",tv.tv_sec, tv.tv_usec);

   if (argc != 2) {
      printf("syntax: %s IFNAME\n", argv[0]);
      exit(1);
   }

   memset(unknown, 0, sizeof(unknown));

   mem_init();

#ifdef NCURS
   //ncurses_init();
   if (!(ncurses_init() == 0))
      return 1;
#endif

   net_init(argv[1]);

   for (;;)
      receive_one();

#ifdef NCURS
   endwin();
#endif

   return 0;
}
