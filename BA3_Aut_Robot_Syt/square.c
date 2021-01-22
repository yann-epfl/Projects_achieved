/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

#include <stdbool.h>

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
  double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv,camsrv;

symTableElement * getinputref (const char *sym_name, symTableElement * tab)
{
  int i;
  for (i=0; i< getSymbolTableSize('r'); i++)
    if (strcmp (tab[i].name,sym_name) == 0)
      return &tab[i];
  return 0;
}

symTableElement * getoutputref (const char *sym_name, symTableElement * tab)
{
  int i;
  for (i=0; i< getSymbolTableSize('w'); i++)
    if (strcmp (tab[i].name,sym_name) == 0)
      return &tab[i];
  return 0;
}
/*****************************************
* odometry
*/
#define WHEEL_DIAMETER   0.06522	/* m */
#define WHEEL_SEPARATION 0.26	/* m */
#define Eb 0.9636
#define Ed 1
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	24902 //24902 (rob)  //8000 (sim)

#define NBLS 8
#define NBIRS 5
#define EPSILON 0.1
#define LTEMPSENS_SIZE 10
#define MOVE_AV_CM 10

#define DISTANCE_MAX 10

#define BLACKL_SENSITIVITY 0.15 //sim 0.2
#define CROSS_BLACK_NB 5
#define FIND_BLACK_NB 1
#define THRESHOLD_DELTA 0.15

#define K1 0.5		//followline correcting factor
#define K2 0.1		//followedge correcting factor 0.2(w) 0.3(b)	
#define FLLWLINE_SPEED 0.2
#define FLLWEDGE_SPEED 0.2
#define FWD_SPEED 0.3
#define TURN_SPEED 0.2
#define FLLWWALL_SPEED 0.3
#define OFFSETir 0.21

/*val sim :

	K1 0.1
	K2 0.2
	FLLWLINE_SPEED 0.4
	FLLWEDGE_SPEED 0.3
*/

/*val reelle :

	K1 0.7
	K2 0.2
	FLLWLINE_SPEED 0.2
	FLLWEDGE_SPEED 0.3
*/

typedef struct{
  //input signals
  int left_enc,right_enc; // encoderticks
  // parameters
  double w;	// wheel separation
  double cr,cl;   // meters per encodertick
  //output signals
  double right_pos,left_pos;
  //odo parameters
  double odoX,odoY,theta;
  // internal variables
  int left_enc_old, right_enc_old;
} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);



/********************************************
* Motion control
*/

typedef struct{
  //input
  int cmd;
  int curcmd;
  double speedcmd;

  double left_pos,right_pos;
  // parameters
  double w;
  //output
  double motorspeed_l, motorspeed_r;
  int finished;
  // internal variables
  double startpos;
  
  double dist; //dist to drive
  double angle; //angle to turn
  int color; //color of line to follow
  int dir; //edge to follow
  double radius; //radius to perform in turnr
  double dist_obst; //distance to obstacle to look at
  int pos_IR_sens;
  int pos_laser_sens;
  int comp;
  
  int stop_cond;
}motiontype;
	       
enum {mot_stop=1,mot_move,mot_turn, mot_followline, mot_followedge, mot_turnr, mot_followwall};
enum {black,white}; //motiontype.color
enum {left,right}; //motiontype.dir
enum {drivendist, crossing_bl, bl_found, bl_not_found, IR_dist, laser_dist}; //motiontype.stop_cond
enum {l, fl, fm, fr, r};
enum {sup, inf};

void update_motcon(motiontype *p, odotype *po, double *pt, double *Lsensor, double *IRsensor, double *cmsave, double threshold, bool cross_black, bool see_obst, bool blackl_found, bool blackl_not_found);

int fwd(double dist, double speed,int time, odotype *p, double *px, double *py);
int bwd(double dist, double speed,int time, odotype *p, double *px, double *py);
int turn(double angle, double speed,int time, odotype *p, double *pt);
int turnr(double angle, double speed,int time, odotype *p, double *pt);
int homing(double *pb, double speed,int time, odotype *p, double *px, double *py);


typedef struct{
  int state,oldstate;
  int time;
  int old_nb_miss, nb_miss;	
}smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {ms_init,ms_fwd,ms_turn,ms_bwd,ms_followline, ms_followedge,ms_turnr,ms_end, ms_followwall, ms_measureStart, ms_measureEnd, ms_homing};

int main()
{
  //FILE * fp;
  //fp = fopen("/home/smr/k385/TMsquare/dataLog.dat","w");
  double startX;
  double startY;
  double startM = 0;
  double m_dist;
  double m_offset;
  double startTheta;
  double *px = &startX;
  double *py = &startY;
  double *pm = &startM;
  double *pt = &startTheta;
  int running,arg,time=0;
  double dist=0,angle=0,speed=0;
  int nb_mission = 0;
  double backdist = 0;
  double *pb = &backdist;
  
  double Lsensor[NBLS];
  double IRsensor[NBIRS];
  double Ltempsensor[LTEMPSENS_SIZE][NBLS];
  double mean[NBLS] = {0, 0, 0, 0, 0, 0, 0, 0};
  int time0 = 1;
  double cmsave[MOVE_AV_CM];
  double threshold = THRESHOLD_DELTA;
  
  bool cross_black = false, see_obst = false, blackl_found = false, blackl_not_found = true;
  
  //**********************
    // smoothing cm
    
    for(int i=0; i<MOVE_AV_CM; i++){
       cmsave[i] = 0;
    }
    
    
    //******************************************

  
  // Real Line sensor Calibration

  // smr10
  //double Bmean[] = {53.1568627450980,	53.6715686274510,	52.8186274509804,	53.3627450980392,	53.3235294117647,	54.4754901960784,	54.7941176470588,	55.3627450980392};
  //double Wmean[] = {68.5414634146342,	71.2487804878049,	70.4585365853659,	70.3902439024390,	73.3414634146342,	74.8390243902439,	73.8048780487805,	72.5414634146342};

  //smr 15
  //double Bmean[] = {47.5392156862745,	47.4215686274510,	48.7549019607843,	48.0392156862745,	48.2941176470588,	48.4558823529412,	48.5196078431373,	49.2156862745098};
  //double Wmean[] = {60.2341463414634,	64.1317073170732,	65.8146341463415,	67.0195121951220,	67.7463414634146,	68.2243902439024,	66.4292682926829,	65.4000000000000};
/*
  //smr 8
  double Bmean[] = {65.8829268292683,	62.9317073170732,	55.1024390243902,	57.9560975609756,	54.8585365853659,	58.3512195121951,	68.0975609756098,	55.7365853658537};
  double Wmean[] = {97.5098039215686,	101.519607843137,	71.4068627450980,	78.9950980392157,	69.4754901960784,	81.1323529411765,	113.808823529412,	69.8480392156863};
*/
  double Bmean[] = {54.9118, 55.0833, 55.3922, 55.2892, 55.0980, 55.1961, 55.4069, 55.9951};
  double Wmean[] = {66.3024, 68.5171, 71.0098, 70.9122, 69.0878, 69.2439, 69.7512, 69.2439};


  //smr 1
  //double Bmean[] = {49.82,	49.88,	54.51,	53.14,	56.23,	59.52,	56.37,	52.5};
  //double Wmean[] = {73.37, 77.98, 77.70, 76.89, 133.91, 92.29, 137.72, 94.99}; 
  

  //Simulator*******************************
  //double Bmean[] = {0,0,0,0,0,0,0,0};
  //double Wmean[] = {128,128,128,128,128,128,128,128};
  /*
  // Real IR sensor Calibration SMR 8
  double Ka[] = {14.66, 15.6552 , 16.2877, 13.5513,13.86};
  double Kb[] = {73.99, 43.6226, 52.8161, 71.1678, 78.93};
*/
// Real IR sensor Calibration SMR 8 old
  double Ka[] = {13.14, 13.2253 , 12.9781 , 12.7467, 14.41};
  double Kb[] = {80.71, 45.1771 , 58.4358 , 72.1583, 74.04};
  

  //smr1
  //double Ka[] = {16.75, 17.69, 18.62, 16.75, 16.84};
  //double Kb[] = {65.35, 61.53, 80.24, 49.14, 61.70};
  
  // Simulator IR sensor Calibration
  //double Ka[] = {16, 16, 16, 16, 16};
  //double Kb[] = {76, 76, 76, 76, 76};
  

  /* Establish connection to robot sensors and actuators.*/
  if (rhdConnect('w',"localhost",ROBOTPORT)!='w'){
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE); 
  } 
  
  printf("connected to robot \n");

  if ((inputtable=getSymbolTable('r'))== NULL){
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE); 
  }

  if ((outputtable=getSymbolTable('w'))== NULL){
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE); 
  }

  // connect to robot I/O variables
  lenc=getinputref("encl",inputtable);
  renc=getinputref("encr",inputtable);
  linesensor=getinputref("linesensor",inputtable);
  irsensor=getinputref("irsensor",inputtable);
        
  speedl=getoutputref("speedl",outputtable);
  speedr=getoutputref("speedr",outputtable);
  resetmotorr=getoutputref("resetmotorr",outputtable);
  resetmotorl=getoutputref("resetmotorl",outputtable);
  
  // **************************************************
  //  Camera server code initialization
  //

  /* Create endpoint */
  lmssrv.port=24919;
  strcpy(lmssrv.host,"127.0.0.1");
  strcpy(lmssrv.name,"laserserver");
  lmssrv.status=1;
  camsrv.port=24920;
  strcpy(camsrv.host,"127.0.0.1");
  camsrv.config=1;
  strcpy(camsrv.name,"cameraserver");
  camsrv.status=1;

  if (camsrv.config) {
    int errno = 0; 
    camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if ( camsrv.sockfd < 0 ){
      perror(strerror(errno));
      fprintf(stderr," Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&camsrv);

    xmldata=xml_in_init(4096,32);
    printf(" camera server xml initialized \n");
  }   
   
   
  // **************************************************
  //  LMS server code initialization
  //

  /* Create endpoint */
  lmssrv.config=1;
  if (lmssrv.config) {
    char buf[256];
    int errno = 0,len; 
    lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if ( lmssrv.sockfd < 0 ){
      perror(strerror(errno));
      fprintf(stderr," Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&lmssrv);
    
    if (lmssrv.connected){
      xmllaser=xml_in_init(4096,32);
      printf(" laserserver xml initialized \n");
      //len=sprintf(buf,"push  t=0.2 cmd='mrcobst width=0.4'\n");
      len=sprintf(buf,"scanpush cmd='zoneobst'\n");
      send(lmssrv.sockfd,buf,len,0);
    }
  }   
   
 
  /* Read sensors and zero our position.*/
  rhdSync();
  
  odo.w=WHEEL_SEPARATION*Eb;
  odo.cr=DELTA_M/(1-((1-Ed)/2));
  odo.cl=DELTA_M*(1-((1-Ed)/2));
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w=odo.w;
  running=1; 
  mission.state=ms_init;
  mission.oldstate=-1;
  mission.nb_miss = 0;
  mission.old_nb_miss = -1;

  //init line sensor
    for(int i=0; i<NBLS; i++){
       for(int j=0; j<LTEMPSENS_SIZE; j++)
       {
          Ltempsensor[j][i] = (linesensor->data[i]-Bmean[i])/(Wmean[i]-Bmean[i]);
       }
    }
    

enum {c_mission_state, c_distance, c_angle, c_speed, c_stop_cond, c_color, c_dir, c_radius, c_dist_obst, c_pos_IR_sens, c_pos_laser_sens, c_comp};
//********************************************************************************
// mission plan :

double mission_plan[][12] = {
	{ms_init, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	// Odometry calibration
/*
	{ms_fwd, 2, 0, FLLWLINE_SPEED, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, 90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 2, 0, FLLWLINE_SPEED, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, 90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 2, 0, FLLWLINE_SPEED, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, 90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 2, 0, FLLWLINE_SPEED, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, 90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},


	{ms_fwd, 2, 0, FLLWLINE_SPEED, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, -90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 2, 0, FLLWLINE_SPEED, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, -90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 2, 0, FLLWLINE_SPEED, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, -90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 2, 0, FLLWLINE_SPEED, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, -90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_end, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}*/
	
	//Measure box
	{ms_measureStart, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_followline, 0.8, 0, 0.1, drivendist, black, 0, 0, 0, 0, 0, 0},
	{ms_followedge, 0.4, 0, 0.1, drivendist, black, right, 0, 0, 0, 0, 0},
	{ms_followline, DISTANCE_MAX, 0, 0.2, IR_dist, black, 0, 0, 0.2, fm, 0, inf},
	{ms_measureEnd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_homing, DISTANCE_MAX, 0, FWD_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, 90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, DISTANCE_MAX, 0, FLLWLINE_SPEED, bl_found, black, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 0.2, 0, FLLWLINE_SPEED, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, DISTANCE_MAX, 0, FLLWLINE_SPEED, bl_found, black, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 0.15, 0, 0.3, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, -90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},

	// box
	{ms_followline, DISTANCE_MAX, 0, 0.3, IR_dist, black, 0, 0, 0.2, fm, 0, inf},
	{ms_followline, DISTANCE_MAX, 0, 0.1, crossing_bl, black, 0, 0, 0, 0, 0, 0},
	{ms_followline, 0.23, 0, 0.2, drivendist, black, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, 90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_bwd, 0.6, 0, 0.2, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_followline, DISTANCE_MAX, 0, 0.2, crossing_bl, black, 0, 0, 0, 0, 0, 0},
	{ms_followline, 0.2, 0, FLLWLINE_SPEED, drivendist, black, 0, 0, 0, 0, 0, 0},
	{ms_followline, DISTANCE_MAX, 0, 0.15, crossing_bl, black, right, 0, 0, 0, 0, 0},
	
	// gate and wall
	{ms_fwd, 0.3, 0, FWD_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, -90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 0.25, 0, 0.15, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, 90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, DISTANCE_MAX, 0, 0.1, IR_dist, black, 0, 0, 0.4, r, 0, inf},
	{ms_fwd, 0.40, 0, 0.2, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, -90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, DISTANCE_MAX, 0, 0.3, IR_dist, black, 0, 0, 0.2, fm, 0, inf},
	{ms_turn, DISTANCE_MAX, -90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_followwall, DISTANCE_MAX, 0, 0.2, bl_found, black, 0, 0.15, 0, l, 0, 0},
	{ms_followwall, 0.4, 0, FLLWWALL_SPEED, drivendist, black, 0, 0.3, 0, l, 0, 0},
	{ms_followline, 0.55, 0, FLLWLINE_SPEED, drivendist, black, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, 180, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_followline, DISTANCE_MAX, 0, FLLWLINE_SPEED, crossing_bl, black, 0, 0, 0, 0, 0, 0},

	// white line and gates
	{ms_followline, DISTANCE_MAX, 0, FLLWLINE_SPEED, crossing_bl, black, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 0.2, 0, FWD_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, 90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 0.2, 0, 0.15, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_followline, 0.5, 0, 0.2, drivendist, white, 0, 0, 0, 0, 0, 0},
	{ms_followline, 0.65, 0, 0.4, drivendist, white, 0, 0, 0, 0, 0, 0},
	{ms_followline, 0.8, 0, 0.2, drivendist, white, 0, 0, 0, 0, 0, 0},
	{ms_followline, 0.4, 0, 0.4, drivendist, white, 0, 0, 0, 0, 0, 0},
	{ms_followline, DISTANCE_MAX, 0, 0.2, bl_found, white, 0, 0, 0, 0, 0, 0},
	{ms_followline, DISTANCE_MAX, 0, 0.2, crossing_bl, black, 0, 0, 0, 0, 0, 0},
	{ms_followline, 0.15, 0, 0.2, drivendist, black, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, 90, TURN_SPEED, drivendist, 0, 0, 0, 0, 0, 0, 0},
	
	// parking
	{ms_followline, 2.5, 0, FLLWLINE_SPEED, drivendist, black, 0, 0, 0, 0, 0, 0},
	{ms_followline, DISTANCE_MAX, 0, FLLWLINE_SPEED, IR_dist, black, 0, 0, 0.2, fm, 0, inf},
	{ms_turn, DISTANCE_MAX, 90, 0.2, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 0.65, 0, 0.3, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, -90, 0.2, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 0.3, 0, 0.3, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, -130, 0.3, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_fwd, 0.35, 0, 0.2, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_turnr, DISTANCE_MAX, 15, 0.2, drivendist, 0, 0, 0.9, 0, 0, 0, 0},
	{ms_turn, DISTANCE_MAX, 80, 0.2, drivendist, 0, 0, 0, 0, 0, 0, 0},
	{ms_followline, 0.2, 0, 0.2, drivendist, black, 0, 0, 0, 0, 0, 0},
	{ms_followline, DISTANCE_MAX, 0, FLLWLINE_SPEED, IR_dist, black, 0, 0, 0.2, fm, 0, inf},
	
	{ms_end, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

//test
/*
double mission_plan[][12] = {
	{ms_init, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ms_followline, 3, 0, FLLWLINE_SPEED, drivendist, white, 0, 0, 0, 0, 0, 0},
	{ms_followedge, 1, 0, FLLWEDGE_SPEED, drivendist, white, right, 0, 0, 0, 0, 0},
	{ms_followline, 1, 0, FLLWLINE_SPEED, drivendist, white, 0, 0, 0, 0, 0, 0},
	{ms_end, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
*/
//{ms_followwall, DISTANCE_MAX, 0, 0.2, crossing_bl, black, left, 0, 0.3, r, 0, 0},
 
//*********************************************************************************


  while (running){ 
    //Line sensor mapping
    //********************************
    int nb_black = 0;
    
    // push the values
    for(int i=0; i<NBLS; i++){
       for(int j=0; j<LTEMPSENS_SIZE-1; j++)
       {
          Ltempsensor[j+1][i] = Ltempsensor[j][i];
       }
       mean[i]=0;
    }    
    
    // refresh the new first value
    for(int i=0; i<NBLS; i++){
      Ltempsensor[0][i] = (linesensor->data[i]-Bmean[i])/(Wmean[i]-Bmean[i]);
      if(Ltempsensor[0][i] < BLACKL_SENSITIVITY)
         nb_black++;
    }
   
    for(int i=0; i<NBLS; i++){
       for(int j=0; j<LTEMPSENS_SIZE; j++)
       { 
          mean[i] = mean[i] + Ltempsensor[j][i]; // push
       }
       mean[i] = mean[i]/10;
    }
    
    for(int i=0; i<NBLS; i++){
      Lsensor[i] = mean[NBLS-1-i];
      //printf("%lf ", Lsensor[i]);
    }
    //printf("\n");
    
        
    if(nb_black > CROSS_BLACK_NB)
    	cross_black = true;
    else
    	cross_black = false;
    	
    if(nb_black > FIND_BLACK_NB)
    	blackl_found = true;
    else
    	blackl_found = false;
    	
    if(nb_black == 0)
    	blackl_not_found = true;
    else
    	blackl_not_found = false;
    
    //*******************************   
    
   //init IR sensor
    for(int i=0; i<NBIRS; i++){
       IRsensor[i] = Ka[i]/((irsensor->data[i])-Kb[i]);
    }
    
    if (lmssrv.config && lmssrv.status && lmssrv.connected){
      while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
        xml_proca(xmllaser);
    }
        
    if (camsrv.config && camsrv.status && camsrv.connected){
      while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
        xml_proc(xmldata);
    }

    rhdSync();
    odo.left_enc=lenc->data[0];
    odo.right_enc=renc->data[0];
    update_odo(&odo);
      
    /****************************************
    / mission statemachine   
    */
            
    sm_update(&mission);
    
    switch (mission.state) {
      case ms_init:
        printf("init\n");
        nb_mission++;      
      break;
    
      case ms_fwd:
        if(time0){
           printf("fwd %lf\n", mission_plan[nb_mission][c_distance]);
           mot.cmd=mot_move;
           time0 = 0;
        }        
        if (fwd(dist,speed,mission.time, &odo, px, py))
        {
          nb_mission++;
          time0 = 1;
        }
      break;
      
      case ms_bwd:
        if(time0){
	   printf("back dist: %f \n", *pb);
           printf("bwd %lf\n", mission_plan[nb_mission][c_distance]);
           mot.cmd=mot_move;
           time0 = 0;
        }        
        if (bwd(dist,speed,mission.time, &odo, px, py))
        {
          nb_mission++;
          time0 = 1;
        }
      break;

      case ms_homing:
        if(time0){
	   //printf("back dist: %f \n", *pb);
           printf("Homing %lf\n", mission_plan[nb_mission][c_distance]);
           mot.cmd=mot_move;
           time0 = 0;
        }        
        if (bwd(*pb,speed,mission.time, &odo, px, py))
        {
          nb_mission++;
          time0 = 1;
        }
      break;
    
      case ms_turn:
        if(time0){
           printf("turn %lf\n", mission_plan[nb_mission][c_angle]);
           mot.cmd=mot_turn;
           time0 = 0;
        }
        if (turn(angle,speed,mission.time, &odo, pt)){
          nb_mission++;
          time0=1;
        }
      break;
      
      case ms_turnr:
        if(time0){
           printf("turnr r : %lf, angle : %lf\n", mission_plan[nb_mission][c_radius], mission_plan[nb_mission][c_angle]);
           mot.cmd=mot_turnr;
           time0 = 0;
        }
        if (turnr(angle,speed,mission.time, &odo, pt)){ 
          nb_mission++;
          time0=1;
        }
      break;
      
      case ms_followline:
        if(time0){
           printf("followline %lf\n", mission_plan[nb_mission][c_distance]);
           mot.cmd=mot_followline;
           time0 = 0;
        }  
        if (fwd(dist,speed,mission.time, &odo, px, py)){
          nb_mission++;
          time0 = 1;
        }
      break;    
      
      case ms_followedge:
        if(time0){
           printf("followedge %lf\n", mission_plan[nb_mission][c_distance]);
           mot.cmd=mot_followedge;
           time0 = 0;
        }  
        if (fwd(dist,speed,mission.time, &odo, px, py)){
          nb_mission++; 
          time0 = 1;
        }
      break; 
      
      case ms_followwall:
        if(time0){
           printf("followwall %lf\n", mission_plan[nb_mission][c_distance]);
           mot.cmd=mot_followwall;
           time0 = 0;
        }  
        if (fwd(dist,speed,mission.time, &odo, px, py)){
          nb_mission++; 
          time0 = 1;
        }
      break;

      case ms_measureStart:
          if(time0){
             printf("measureStart\n");
             *pm = -odo.odoY;
             printf("%f \n",*pm);
             mot.cmd=mot_move;
             time0 = 0;
         }
        
        else{
           nb_mission++; 
           time0 = 1;
        }
      break;
      
      case ms_measureEnd:
        if(time0){
           printf("measureEnd\n");
           mot.cmd=mot_move;
           m_offset = IRsensor[fm];
           m_dist = (-odo.odoY-*pm + m_offset + OFFSETir-0.04)/0.97;
	   //printf("y: %f \n", -odo.odoY);
           printf("distance: %.2f \n",round(m_dist*100)/100);
           //printf("offset: %f \n", m_offset);
           time0 = 0;
           *pb = (m_dist-1.3-m_offset-OFFSETir);
	   printf("back dist: %f \n", *pb);
        }
        else{
           nb_mission++; 
           time0 = 1;
        }
      break;

      case ms_end:
        printf("end\n");
        mot.cmd=mot_stop;
        running=0;
      break;
    }  
    /*  end of mission  */
        if(time0){
	    mission.state = mission_plan[nb_mission][c_mission_state];
	    dist = mission_plan[nb_mission][c_distance];
	    angle = mission_plan[nb_mission][c_angle];
	    speed = mission_plan[nb_mission][c_speed];
	    mot.stop_cond = mission_plan[nb_mission][c_stop_cond];
	    mot.color = mission_plan[nb_mission][c_color];
	    mot.dir = mission_plan[nb_mission][c_dir];
	    mot.radius = mission_plan[nb_mission][c_radius];
	    mot.dist_obst = mission_plan[nb_mission][c_dist_obst];
	    mot.pos_IR_sens = mission_plan[nb_mission][c_pos_IR_sens]; 
	    mot.pos_laser_sens = mission_plan[nb_mission][c_pos_laser_sens];
	    mot.comp = mission_plan[nb_mission][c_comp];
    }
    
    mission.nb_miss = nb_mission; 
  
    mot.left_pos=odo.left_pos;
    mot.right_pos=odo.right_pos;
    update_motcon(&mot, &odo, pt, Lsensor, IRsensor, cmsave, threshold, cross_black, see_obst, blackl_found, blackl_not_found);
    speedl->data[0]=100*mot.motorspeed_l;
    speedl->updated=1;
    speedr->data[0]=100*mot.motorspeed_r;
    speedr->updated=1;
    if (time  % 100 ==0)
      time++;
    /* stop if keyboard is activated
    *
    */
    ioctl(0, FIONREAD, &arg);
    if (arg!=0)
      running=0;
  }
  
  /* end of main control loop */
  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
  exit(0);
  //fclose(fp);
}


/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */


void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  p->odoX = p->theta = 0.0;
  p->odoY = 0.0;
}

void update_odo(odotype *p)
{
  int delta;

  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta * p->cr;
  
  double dur = delta * p->cr;

  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta * p->cl;
  
  double dul = delta * p->cl;

  p->theta += ((dur-dul)/p->w)*180/M_PI;
  double dui = (dur+dul)/2;
  p->odoX += dui * cos((p->theta)*M_PI/180);
  p->odoY += dui * sin((p->theta)*M_PI/180);
}


void update_motcon(motiontype *p, odotype *po, double *pt, double *Lsensor,double *IRsensor, double *cmsave, double threshold, bool cross_black, bool see_obst, bool blackl_found, bool blackl_not_found)
{ 
  double cm_n = 0;
  double cm_d = 0;
  double cm = 0 ;
  double deltas[NBLS-1];
  int lbound = 0, rbound = 7, i_edge = 0;
  double mean_cm = 0;
  double vlow = 0;
  double vhigh = 0;
  
  bool stop = false;

  if (p->cmd !=0)
  {
    p->finished=0;
    switch (p->cmd){
      case mot_stop:
        p->curcmd=mot_stop;
      break;

      case mot_move:
        p->startpos=(p->left_pos+p->right_pos)/2;
        p->curcmd=mot_move;
      break;

      case mot_followline:
        p->startpos=(p->left_pos+p->right_pos)/2;
        p->curcmd=mot_followline;
      break;

      case mot_followedge:
        p->startpos=(p->left_pos+p->right_pos)/2;
        p->curcmd=mot_followedge;
      break;
      
      case mot_followwall:
        p->startpos=(p->left_pos+p->right_pos)/2;
        p->curcmd=mot_followwall;
      break;
      
      case mot_turn:
        if (p->angle > 0) 
          p->startpos=p->right_pos;
        else
          p->startpos=p->left_pos;
        p->curcmd=mot_turn;
      break;
      
      case mot_turnr:
        if (p->angle > 0) 
          p->startpos=p->right_pos;
        else
          p->startpos=p->left_pos;
        p->curcmd=mot_turnr;
      break;
    }
    p->cmd=0;
  }
     
  switch (p->stop_cond){
    case crossing_bl:
      if(cross_black){
         stop = true;
         }
    break;
    
    case bl_found:
      if(blackl_found){
         stop = true;
         }
    break;
    
    case bl_not_found:
      if(blackl_not_found){
         stop = true;
         }
    break;
    
    case IR_dist:  
      if(p->comp == sup && (IRsensor[p->pos_IR_sens]) > p->dist_obst)
         stop = true;
      if(p->comp == inf && fabs(IRsensor[p->pos_IR_sens]) < p->dist_obst)
         stop = true;
    break;
    
    case laser_dist:
      if(p->comp == sup && laserpar[p->pos_laser_sens] > p->dist_obst)
         stop = true;
      if(p->comp == inf && laserpar[p->pos_laser_sens] < p->dist_obst)
         stop = true;
    break;
  }
    
  switch (p->curcmd){
    case mot_stop:
      p->motorspeed_l=0;
      p->motorspeed_r=0;
    break;

    case mot_move:
      
      if (fabs((p->right_pos+p->left_pos)/2- p->startpos) > p->dist || stop){
        p->finished=1;
        p->motorspeed_l=0;
        p->motorspeed_r=0;
      }
      else {	
        p->motorspeed_l=p->speedcmd*(1-((1-Ed)/2));
        p->motorspeed_r=p->speedcmd/(1-((1-Ed)/2));
      }
    break;

    case mot_followline:
	//printf("dist : %lf \n",IRsensor[p->pos_IR_sens]);
      //calc cm
      if(p->color == white){
        for(int i = 0; i<NBLS; i++)
        {
          cm_n += (Lsensor[i])*(i-3.5)/2;
          cm_d += (Lsensor[i]);
        }
      }
      else{
        for(int i = 0; i<NBLS; i++)
        {
          cm_n += (1-Lsensor[i])*(i-3.5);
          cm_d += (1-Lsensor[i]);
        }
      }
      cm = cm_n/cm_d;
      
      //printf("cm : %lf\n", cm);  
      
      if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist || stop)
      {
        p->finished=1;
        p->motorspeed_l=0;
        p->motorspeed_r=0;
      }	  
      else
      {	
        p->motorspeed_l=(p->speedcmd)+(cm*K1);
        p->motorspeed_r=(p->speedcmd)-(cm*K1);
      }
      
      //printf("p->speedcmd : %lf, cm : %lf\n", p->speedcmd, cm);
      //printf("p->motorspeed_l : %lf, p->motorspeed_r : %lf\n", p->motorspeed_l, p->motorspeed_r);
      
    break;

    case mot_followedge:
      
      if(p->color == white){
    	if(p->dir == right){
      		for(int i = rbound-1; i >= lbound; i--){
        		deltas[i] = Lsensor[i+1]-Lsensor[i];
        		if(deltas[i] < (-threshold)){
        			i_edge = i-3;
        			break;
        		}
        	}
        }
        else{
        	for(int i = lbound; i < rbound; i++){
        		deltas[i] = Lsensor[i+1]-Lsensor[i];
        		if(deltas[i] > threshold){
        			i_edge = i-3;
        			break;
        		}
        	}
        }
      }
      else{
      	if(p->dir == right){
      		for(int i = rbound-1; i >= lbound; i--){
        		deltas[i] = Lsensor[i+1]-Lsensor[i];
        		if(deltas[i] > threshold){
        			i_edge = i-3;
        			break;
        		}
        	}
        }
        else{
        	for(int i = lbound; i < rbound; i++){
        		deltas[i] = Lsensor[i+1]-Lsensor[i];
        		if(deltas[i] < (-threshold)){
        			i_edge = i-3;
        			break;
        		}
        	}
        }
      }
      cm = i_edge;
      
      //************ update cm
       for(int i=MOVE_AV_CM-1; i>0; i--)
       {
          cmsave[i] = cmsave[i-1]; // push
       }
       
       cmsave[0] = cm;
       
       for(int i=0; i<MOVE_AV_CM; i++)
       {
          mean_cm += cmsave[i]; // push
       }
       mean_cm = mean_cm/MOVE_AV_CM;
       cm = mean_cm;
      //*******************  
      
      if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist || stop)
      {     
        p->finished=1;
        p->motorspeed_l=0;
        p->motorspeed_r=0;
      }	  
      else
      {	
        p->motorspeed_l=(p->speedcmd)+(cm*K2);
        p->motorspeed_r=(p->speedcmd)-(cm*K2);
      }
    break;
    
    case mot_followwall:
    
		if(p->pos_IR_sens == r){
			cm=0.8*(p->radius-IRsensor[p->pos_IR_sens]);
		}
		else{
			cm=-0.8*(p->radius-IRsensor[p->pos_IR_sens]);
		}
//          printf("dist : %lf \n",IRsensor[p->pos_IR_sens]);
    if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist || stop){
       p->finished=1;
       p->motorspeed_l=0;
       p->motorspeed_r=0;
     }
    else{
    	if (cm > 0.4*p->speedcmd){
    	cm=0.4*p->speedcmd;
	
    	}
    	if (cm < -0.4*p->speedcmd){
    	cm=-0.4*p->speedcmd;
    	}
	if (IRsensor[p->pos_IR_sens] < 0 && p->pos_IR_sens == r){
	  cm=-0.4*p->speedcmd;
	}
	if (IRsensor[p->pos_IR_sens] < 0 && p->pos_IR_sens == l){
	  cm=0.4*p->speedcmd;
	}
      p->motorspeed_l=(p->speedcmd)-(cm);
      p->motorspeed_r=(p->speedcmd)+(cm);
      //printf("dist : %lf \n",IRsensor[p->pos_IR_sens]);
    }
    break;
    
    case mot_turn:
    
      if (p->angle>0)
      {
        if ((po->theta-*pt < (p->angle*180/M_PI)-EPSILON) && !stop)
        {
          p->motorspeed_l=-(p->speedcmd);
          p->motorspeed_r=(p->speedcmd);
        }
        else
        {
          //printf("real angle : %lf / command : %lf \n",po->theta-*pt,p->angle*180/M_PI);	     
          p->motorspeed_r=0;
          p->motorspeed_l=0;
          p->finished=1;
        }
      }
      else
      {
        if ((po->theta-*pt > ((p->angle)*180/M_PI)-EPSILON) && !stop){
          p->motorspeed_r=-(p->speedcmd);
          p->motorspeed_l=(p->speedcmd);
        }
        else
        {	
          //printf("real angle : %lf / command : %lf \n",po->theta-*pt,p->angle*180/M_PI);	
          p->motorspeed_r=0;     
          p->motorspeed_l=0;
          p->finished=1;
        }
      }
    break;
    
    case mot_turnr:
    
      vlow = (p->speedcmd)*((p->radius)-WHEEL_SEPARATION/2)/(p->radius);
      vhigh = (p->speedcmd)*((p->radius)+WHEEL_SEPARATION/2)/(p->radius);
      
      //printf("speedcmd : %lf, radius : %lf, vhigh : %lf, low : %lf\n",(p->speedcmd), (p->radius), vhigh, vlow);
      
      if (p->angle>0)
      {
        if ((po->theta-*pt < (p->angle)-EPSILON) && !stop)
        {
          //printf("real angle : %lf / command : %lf \n",po->theta-*pt,p->angle);
          p->motorspeed_l=vlow;
          p->motorspeed_r=vhigh;
        }
        else
        {
          //printf("stop : real angle : %lf / command : %lf \n",po->theta-*pt,p->angle);	     
          p->motorspeed_r=0;
          p->motorspeed_l=0;
          p->finished=1;
        }
      }
      else
      {
        if ((po->theta-*pt > ((p->angle))-EPSILON) && !stop){
          p->motorspeed_r=vlow;
          p->motorspeed_l=vhigh;
        }
        else
        {	
          //printf("real angle : %lf / command : %lf \n",po->theta-*pt,p->angle);	
          p->motorspeed_r=0;     
          p->motorspeed_l=0;
          p->finished=1;
        }
      }
    break;
  }   
}   


int fwd(double dist, double speed,int time, odotype *p, double *px, double *py)
{
  if (time==0){
    *px = p->odoX;
    *py = p->odoY;
    mot.speedcmd=0.005;
    mot.dist=dist;
    return 0;
  }
  else
  {
    double rDist = dist - sqrt(pow((p->odoX)-(*px),2)+pow((p->odoY)-(*py),2));
    double Vmax = sqrt(rDist);
    if (mot.speedcmd+0.005<speed){
      if (mot.speedcmd+0.005<Vmax){
        mot.speedcmd += 0.005;
      }
      else{
        mot.speedcmd = Vmax;
      }
    }
    else{
      if (speed<Vmax){
        mot.speedcmd = speed;
      }
      else{
        if (Vmax > 0.0){
          mot.speedcmd = Vmax;
        }
        else{
          mot.speedcmd = 0.014;
        }
      }
    }
  }
  return mot.finished;
}

int bwd(double dist, double speed,int time, odotype *p, double *px, double *py)
{
  if (time==0){
    *px = p->odoX;
    *py = p->odoY;
    mot.speedcmd=-0.005;
    mot.dist=dist;
    return 0;
  }
  else
  {
    double rDist = dist - sqrt(pow((p->odoX)-(*px),2)+pow((p->odoY)-(*py),2));
    double Vmax = -sqrt(rDist);
    if (mot.speedcmd-0.005>-speed){
      if (mot.speedcmd-0.005>Vmax){
        mot.speedcmd -= 0.005;
      }
      else{
        mot.speedcmd = Vmax;
      }
    }
    else{
      if (-speed>Vmax){
        mot.speedcmd = -speed;
      }
      else{
        if (Vmax < 0.0){
          mot.speedcmd = Vmax;
        }
        else{
          mot.speedcmd = -0.014;
        }
      }
    }
  }
  return mot.finished;
}

int homing(double *pb, double speed,int time, odotype *p, double *px, double *py)
{
  if (time==0){
    *px = p->odoX;
    *py = p->odoY;
    mot.speedcmd=-0.005;
    mot.dist=*pb;
    return 0;
  }
  else
  {
    double rDist = *pb - sqrt(pow((p->odoX)-(*px),2)+pow((p->odoY)-(*py),2));
    double Vmax = -sqrt(rDist);
    if (mot.speedcmd-0.005>-speed){
      if (mot.speedcmd-0.005>Vmax){
        mot.speedcmd -= 0.005;
      }
      else{
        mot.speedcmd = Vmax;
      }
    }
    else{
      if (-speed>Vmax){
        mot.speedcmd = -speed;
      }
      else{
        if (Vmax < 0.0){
          mot.speedcmd = Vmax;
        }
        else{
          mot.speedcmd = -0.014;
        }
      }
    }
  }
  return mot.finished;
}

int turn(double angle, double speed,int time, odotype *p, double *pt){
/*
   if (time==0){ 
     mot.cmd=mot_turn;
     mot.speedcmd=speed;
     mot.angle=angle;
     return 0;
   }
*/

   angle = angle*M_PI/180;

   if (time==0){
     *pt = p->theta;
     mot.speedcmd=0.005;
     mot.angle=angle;
     return 0;
   }
   else{
   	 double rAngle = fabs(angle*180/M_PI - (p->theta-*pt));
	 double rDist = M_PI*p->w*rAngle/360;
	 double Vmax = sqrt(rDist);
     if (mot.speedcmd+0.005<speed){
       if (mot.speedcmd+0.005<Vmax){
         mot.speedcmd += 0.005;
       }
       else{
       	 if (Vmax > 0.00){
         mot.speedcmd = Vmax;
         }
         else{
         	mot.speedcmd = 0.02;
         }
       }
     }
     else{
     	if (speed<Vmax){
     		mot.speedcmd = speed;
     	}
     	else{
     		if (Vmax > 0.00){
     		mot.speedcmd = Vmax;
     		}
     		else{
     			mot.speedcmd = 0.02;
     		}
     	}
     }
   }
     return mot.finished;
}

int turnr(double angle, double speed,int time, odotype *p, double *pt){
/*
   if (time==0){ 
     mot.cmd=mot_turn;
     mot.speedcmd=speed;
     mot.angle=angle;
     return 0;
   }
*/
    
   if (time==0){
     *pt = p->theta;
     mot.speedcmd=0.005;
     mot.angle=angle;
     return 0;
   }
   else{
   	 double rAngle = fabs(angle - (p->theta-*pt));
	 double rDist = M_PI*p->w*rAngle/360;
	 double Vmax = sqrt(rDist);
	 
	 //printf("angle : %lf, rAngle : %lf, pt : %lf, pteta : %lf, Vmax : %lf\n", angle, rAngle, *pt, p->theta, Vmax);
	 
     if (mot.speedcmd+0.005<speed){
       if (mot.speedcmd+0.005<Vmax){
         mot.speedcmd += 0.005;
       }
       else{
          if (Vmax > 0.05){
     	     mot.speedcmd = Vmax;
          }
     	  else{
     	     mot.speedcmd = 0.05;
     	  }
       }
     }
     else{
     	if (speed<Vmax){
     		mot.speedcmd = speed;
     	}
     	else{
     		if (Vmax > 0.05){
     		mot.speedcmd = Vmax;
     		}
     		else{
     			mot.speedcmd = 0.05;
     		}
     	}
     }
   }
   
     return mot.finished;
}


void sm_update(smtype *p){
  if (p->nb_miss!=p->old_nb_miss){
       p->time=0;
       p->old_nb_miss=p->nb_miss;
   }
   else {
     p->time++;
   }
}
