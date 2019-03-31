#include "stm32f4xx.h"
#include <math.h>

/* Define */
#define				pi				(double)3.14159265358979
/* Define Type */
typedef enum
{
	Check_NOK = 0,
	Check_OK,
}Check_Status;

typedef enum 
{
	false = 0,
	true,
}bool;

typedef	enum{
	None = 0,
	Vehicle_Config,
	Sample_Time,
	Send_Time,
	IMU_Config,
	Soft_Reset,
	Manual_Config,
	Auto_Config,
	Path_Plan,
	Flash_Save,
	KeyBoard_Control,
}Command_State;

typedef enum{
	None_Mode = 0,
	Auto_Mode,
	Manual_Mode,
	Calib_Mode,
	KeyBoard_Mode,
}Mode;

typedef struct Time{
	uint32_t		Sample_Time;
	uint32_t    Time_Count;
	uint16_t    Send_Time;
	double			T;
}Time;

typedef	struct  DCMotor{
	double		Kp;
	double 		Ki;
	double		Kd;
	uint16_t  Enc;
	uint16_t 	PreEnc;
	uint8_t		OverFlow;
	double		Set_Vel;
	double 		Current_Vel;
	double		*SampleTime;
	double		Pre_PID;
	double    Pre2_Error;
	double    Pre_Error;
	double		PID_Out;
}DCMotor;


typedef struct trimf{
	double a1;
	double a2;
	double a3;
}trimf;

typedef struct trapf{
	double h1;
	double h2;
	double h3;
	double h4;
}trapf;

typedef struct IMU{
	/* Current Angle and Set Angle */
	double		Angle;
	double 		Set_Angle;
	double		Pre_Angle;
	/* Fuzzy input and output */
	double		Fuzzy_Out;
	double		Fuzzy_Error;
	double 		Fuzzy_Error_dot;
	/* Variables Ke, Kedot and Ku */
	double		Ke;
	double  	Kedot;
	double 		Ku;
	bool			First_RxFlag;
}IMU;

typedef struct GPS{
	double 		CorX;
	double 		CorY;
	IMU				*Angle;
	double		Latitude;
	double    Longitude;
	double 		Pre_CorX;
	double 		Pre_CorY;
	double 		Robot_Velocity;
	bool 			Rx_Flag;
	bool			Send_Flag;
	int 			NbOfWayPoints;
	double		Delta_Angle;
	double    Path_X[20];
	double		Path_Y[20];
	double 		Path_Yaw[20];
}GPS;


typedef	struct Vehicle
{
	Mode								Mode;
	int 								LengthOfCommands;
	char								ManualCtrlKey;
	double    					Max_Velocity;
	double							Manual_Velocity;
	double							Manual_Angle;
}Vehicle;

typedef	struct Message
{
	uint8_t RxTempBuffer[100];
	char    Message[50][30];
}Message;

typedef struct FlashMemory{
	uint8_t			 ReadOutBuffer[500];
	uint8_t			 WriteInBuffer[500];
	char  			 Message[20][30];
	int				 	 Length;
}FlashMemory;

#define  					K1 															1/(2*pi)
#define						K2															4/pi
#define						K3															1
#define						Wheel_Radius 										0.085
#define						K																0.5
#define						FLASH_ProgramType_Byte					VoltageRange_1
#define						FLASH_ProgramType_HalfWord			VoltageRange_2
#define						FLASH_ProgramType_Word					VoltageRange_3
#define						FLASH_ProgramType_DoubleWord		VoltageRange_4
#define						FLASH_PIDPara_BaseAddr 					0x08060000	  //(4 KBytes) (0x08060000 - 0x08060FFF)
#define						FLASH_FuzPara_BaseAddr					0x08061000		//(4 Kbytes) (0x08061000 - 0x08061FFF)
#define						FLASH_GPSPara_BaseAddr					0x08062000		//(120 KBytes) 
/* Export variables */
extern DCMotor 										M1,M2;
extern Vehicle										Veh;
extern trimf 											In1_NS,In1_ZE,In1_PS,In2_ZE;
extern trapf 											In1_NB,In1_PB,In2_NE,In2_PO;
extern double 										NB,NM,NS,ZE,PS,PM,PB;
extern GPS   											GPS_NEO;
extern IMU												Mag;
extern Message  									U2,U6;
extern Time												Timer;
extern FlashMemory 								Flash;
/*--------Export Function------------------------- */
void 	 					LedTest(void);     																				// Use 4 leds on board to test code
double 					ToDegree(double rad); 																		// Convert rad to degree
double 					ToRadian(double degree); 																	// Convert degree to rad
uint8_t 				ToChar(double value, uint8_t *pBuffer); 									// Convert double value to char array
uint8_t					ToHex(uint8_t input);
double					ToRPM(double vel);
double					fix(double value);
double 					Pi_To_Pi(double angle);
/*------------ Timer Function --------------------*/
void						Time_ParametersInit(Time *ptime, uint32_t Sample, uint32_t Send);
void						Time_SampleTimeUpdate(Time *ptime, uint32_t Sample);
void						Time_GetSampleTime(Time *ptime);
void						Time_SendTimeUpdate(Time *ptime, uint32_t TSend);
/*------------ Vehicle status functions ----------*/
void						Veh_ParametersInit(Vehicle *pveh);
void						Veh_GetManualCtrlKey(Vehicle *pveh, char key);
void						Veh_UpdateVehicleFromKey(Vehicle *pveh);
void						Veh_UpdateMaxVelocity(Vehicle *pveh, double MaxVelocity);
/*------------ PID Function ----------------------*/
void						PID_UpdateEnc(DCMotor *ipid, uint16_t PulseCount);
void						PID_SavePIDParaToFlash(FlashMemory *pflash, DCMotor *M1, DCMotor *M2);
void 						PID_Compute(DCMotor *ipid);
void 						PID_ParametersInitial(DCMotor *ipid);
void					  PID_UpdateSetVel(DCMotor *ipid, double SetVal);
void 						PID_ParametersUpdate(DCMotor *ipid, double Kp, double Ki, double Kd);
void						PID_ResetEncoder(DCMotor *ipid);
/* -------Send and Receive data function------------ */
void 						GetMessageInfo(char *inputmessage, char result[50][30], char character);
double 					GetValueFromString(char *value);
int 						Readline(uint8_t *inputmessage, uint8_t *outputmessage);
uint8_t			 		LRCCalculate(uint8_t *pBuffer, int length);
Check_Status  	IsValidData(char input);
Check_Status 		IsCorrectMessage(uint8_t *inputmessage, int length, uint8_t byte1, uint8_t byte2);
Check_Status		StringHeaderCompare(char *s1, char *s2);
Command_State		GetNbOfReceiveHeader(char *input);
int							FeedBack(uint8_t *outputmessage, char status);

/*--------Stanley functions and GPS --------------*/
void						GPS_ParametersInit(GPS *pgps);
void 						GPS_StanleyControl(GPS *pgps);
double					GPS_LLToDegree(double LL);
void 						GPS_LatLonToUTM(GPS *pgps);  // Get 2 values of lat lon and update UTM coordiante to Corx and Cory
void	 					GPS_GetLatFromString(GPS *pgps, char *inputmessage);
void						GPS_GetLonFromString(GPS *pgps, char *inputmessage);
void						GPS_UpdatePathYaw(GPS *pgps);
void						GPS_UpdatePathCoordinate(GPS *pgps, uint8_t *inputmessage);
void						GPS_SavePathCoordinateToFlash(GPS *pgps, FlashMemory *pflash);

/*--------Fuzzy control-------------------*/
void						Fuzzy_ParametersInit(void);
double 					Trapf(trapf *ptrapf, double x);
double 					Trimf(trimf *ptrimf, double x);
void						Trimf_Update(trimf *ptrimf, double a1, double a2, double a3);
void						Trapf_Update(trapf *ptrapf, double a1, double a2, double a3, double a4);
void						Defuzzification_Max_Min(IMU *pimu);

/*--------IMU functions ---------*/
void						IMU_ParametesInit(IMU *pimu);
void						IMU_UpdateSetAngle(IMU *pimu, double *pangle);
void						IMU_UpdatePreAngle(IMU *pimu);
void						IMU_UpdateAngle(IMU *pimu, double Angle);
void						IMU_UpdateFuzzyInput(IMU *pimu, double *pSampleTime);
void						IMU_UpdateFuzzyCoefficients(IMU *pimu, double Ke, double Kedot, double Ku);
double 					IMU_GetValue(uint8_t *inputmessage, int Val);
int 						IMU_GetCommandMessage(char *inputmessage, uint8_t *outputmessage);

/*-------- Flash Memory Embedded functions --------*/
void 						WriteToFlash(FlashMemory *pflash, uint32_t FLASH_Sector, uint32_t FLASH_BaseAddr);
void 						ReadFromFlash(FlashMemory *pflash, uint32_t FLASH_BaseAddr);
void						EraseMemory(uint32_t Flash_Sector);































