#include "stm32f4xx.h"
#include <math.h>
/* Define */
#define				pi				(double)3.1415926535897931
/* Define Type */
typedef enum
{
	Check_NOK = 0,
	Check_OK,
}Check_Status;

struct  DCMotor{
	double  	Kp;
	double 	  Ki;
	double 	  Kd;
	double	  Pre_Error;
	double	  Pre2_Error;
	double 		Pre_PID;
	uint16_t  Enc;
	uint16_t 	PreEnc;
	uint8_t		OverFlow;
	uint32_t 	TotalEnc;
	double 		Velocity;
	double    Duty_Cycle;
}M1,M2;

struct trimf{
	double a1;
	double a2;
	double a3;
}In1_NS,In1_ZE,In1_PS,In2_ZE;

struct trapf{
	double h1;
	double h2;
	double h3;
	double h4;
}In1_NB,In1_PB,In2_NE,In2_PO;

#define  					K1 			1/(2*pi)
#define						K2			4/pi
#define						K3			100
/* Export variables */
extern double		Path_X[100], Path_Y[100];
extern double 	NB,NM,NS,ZE,PS,PM,PB;
/*------------- GPS export variables ------------*/
/* Export Function */
void 	 					LedTest(void);    // Use 4 leds on board to test code
double 					PID_ControlM1(double Current, double SetValue, double T); // Code PID M1
double 					PID_ControlM2(double Current, double SetValue, double T); // Code PID M2
double					PID_AngleControl(double Current, double SetValue, double T); // Code PID angle
double 					ToDegree(double rad); // Convert rad to degree
double 					ToRadian(double degree); // Convert degree to rad
uint8_t 				ToChar(double value, uint8_t *pBuffer); // Convert double value to char array
/* -------GPS read data function------------ */
void	 					LatLonToUTM(double lat, double lon, double *result);
double 					LLToDegree(double LL);
void 	 					GetMessageInfo(char *inputmessage, char result[50][20]);
double 					GetValueFromString(char *value);
Check_Status  	IsCorrectHeader(char *header);
void   					GetMessage(uint8_t *DMA_Buffer, uint8_t *buff);
Check_Status  	IsValidData(char *input);
/*--------Stanley functions --------------*/
double					ToRPM(double vel);
void 						PathAngular(double *X, double *Y, double *result);
double 					Pi_To_Pi(double angle);
double 					StanleyControl(double x, double y, double theta, double *Px, double *Py, double *PYaw, double v);
/*--------Fuzzy control-------------------*/
double 					Fuzzy_Min(double in1, double in2);
double 					Fuzzy_Max(double *input, int len);
double					Input1(double x,char c[2]);
double					Input2(double x,char c[2]);
double					Defuzzification_Max_Min(double in1, double in2);








































