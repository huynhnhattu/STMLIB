#include "functions.h"

/* Global variables */
GPIO_InitTypeDef				 F_GPIO_Struct;
/*----- Stanley Variables -----*/
double 						Path_X[100] = {0};
double					  Path_Y[100] = {0};
int 							NbOfWP = 10;
double 						K = 0.5;
/*----- Robot Parameter -------*/
double            Wheel_Radius = 0.085;   //(m)

/*----- Functions -----*/
/** @brief  : Led test
**	@agr    : void
**	@retval : None
**/
void LedTest(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	F_GPIO_Struct.GPIO_Mode 				= GPIO_Mode_OUT;
	F_GPIO_Struct.GPIO_Pin					= GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	F_GPIO_Struct.GPIO_OType				= GPIO_OType_PP;
	F_GPIO_Struct.GPIO_PuPd					= GPIO_PuPd_NOPULL;
	F_GPIO_Struct.GPIO_Speed				= GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&F_GPIO_Struct);
}
/** @brief  : PID Control M1
**	@agr    : Current position, Set point, Sample time T
**	@retval : PID value
**/
double PID_ControlM1(double CurrentValue, double SetValue, double T)
{
	double 					PID;
	double 					Error, P_Part, I_Part, D_Part;
	Error 					= SetValue - CurrentValue;
	P_Part 					= M1.Kp * (Error - M1.Pre_Error);
	I_Part 					= 0.5 * M1.Ki * T * (Error + M1.Pre_Error);
	D_Part					= M1.Kd / T * (Error - 2 * M1.Pre_Error + M1.Pre2_Error);
	PID 						= M1.Pre_PID + P_Part + I_Part + D_Part;
	M1.Pre2_Error		= M1.Pre_Error;
	M1.Pre_Error		= Error;
	M1.Pre_PID			= PID;
	if(PID < (double)-100)
		PID = (double)-100;
	else if (PID > (double)100)
		PID = (double)100;
	return	PID;
}

/** @brief  : PID Control M2
**	@agr    : Current position, Set point, Sample time T
**	@retval : PID value
**/
double PID_ControlM2(double Current, double SetValue, double T)
{
	double 					PID;
	double					Error, P_Part, I_Part, D_Part;
	Error 					= SetValue - Current;
	P_Part 					= M2.Kp * (Error - M2.Pre_Error);
	I_Part 					= 0.5 * M2.Ki * T * (Error + M2.Pre_Error);
	D_Part					= M2.Kd / T * (Error - 2 * M2.Pre_Error + M2.Pre2_Error);
	PID 						= M2.Pre_PID + P_Part + I_Part + D_Part;
	M2.Pre2_Error		= M2.Pre_Error;
	M2.Pre_Error		= Error;
	M2.Pre_PID			= PID;
	if(PID < (double)-100)
		PID = (double)-100;
	else if (PID > (double)100)
		PID = (double)100;
	return	PID;
}

/** @brief  : Convert rad to degree
**  @agr    : input rad value
**  @retval : degree value
**/
double ToDegree(double rad)
{
	double result;
	result = (rad * 180) / pi;
	return result;
}

/** @brief  : Convert degree to rad
**  @agr    : input degree value
**  @retval : rad value
**/
double ToRadian(double degree)
{
	double result;
	result = (degree * pi) / 180;
	return result;
}

/** @brief  : Round value
**  @agr    : input
**  @retval : Return fix value
**/
double fix(double value)
{
	double result;
	int temp;
	temp = (int)value;
	result = (double)temp;
	return result;
}

/** @brief  : Convert lat lon coordinate into UTM
**  @agr    : input lat and lon values from GNGLL message GLONASS Lat Lon
**  @retval : Result buffer x ,y
**/
void LatLonToUTM(double lattitude, double longitude, double *result)
{
	double la, lo, lat, lon, sa, sb, e2, e2cuadrada, c, Huso, S, deltaS, a, epsilon, nu, v, ta, a1, a2, j2, j4, j6, alfa, beta, gama, Bm, xx, yy;
	la = lattitude;
	lo = longitude;
	sa = 6378137.00000;
	sb = 6356752.314245;
	e2 = pow(pow(sa,2) - pow(sb,2),0.5) / sb;
	e2cuadrada = pow(e2,2);
	c = pow(sa,2) / sb;
	lat = la * (pi / 180);
	lon = lo * (pi / 180);
	Huso = fix((lo / 6) + 31);
	S = ((Huso * 6) - 183);
	deltaS = lon - (S * (pi / 180));
	a = cos(lat) * sin(deltaS);
	epsilon = 0.5 * log((1 + a) / (1 - a));
	nu = atan(tan(lat) / cos(deltaS)) - lat;
	v = (c / pow((1 + (e2cuadrada * pow(cos(lat),2))),0.5)) * 0.9996;
	ta = (e2cuadrada / 2) * pow(epsilon,2) * pow(cos(lat),2);
	a1 = sin(2 * lat);
	a2 = a1 * pow(cos(lat),2);
	j2 = lat + (a1 / 2);
	j4 = ((3 * j2) + a2) / 4;
	j6 = ((5 * j4) + (a2 * pow(cos(lat),2))) / 3;
	alfa = ((double)3 / (double)4) * e2cuadrada;
	beta = ((double)5 / (double)3) * pow(alfa,2);
	gama = ((double)35 / (double)27) * pow(alfa,3);
	Bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);
	xx = epsilon * v * (1 + (ta / 3)) + 500000;
	yy = nu * v * (1 + ta) + Bm;
	if (yy < 0)
	{
		yy = 9999999 + yy;
	}
	result[0] = xx;
	result[1] = yy;
}

/** @brief  : Convert lattitude and longtitude value into Degree
**  @agr    : Lattitude value
**  @retval : Degree value
**/
double LLToDegree(double LL)
{
	double degree, minute, temp;
	degree = (int)(LL / 100);
	temp = (double)(degree * 100);
	minute = LL - temp;
	minute = minute / 60;
	return (degree + minute);
}

/** @brief  : Seperating each info of message by ','
**  @agr    : input message, result buffer
**  @retval : None
**/

void GetMessageInfo(char *inputmessage, char result[50][20])
{
	int col = 0, row = 0, index = 0;
	while(inputmessage[index] != 0)
	{
		if(inputmessage[index] != ',')
		{
			result[row][col] = inputmessage[index];
			col++;
		}
		else 
		{
			row++;
			col = 0;
		}
		index++;
	}
}

/** @brief  : Get double value from string or array
**  @agr    : Input string or array
**  @retval : Double output value
**/
double GetValueFromString(char *value)
{
	char temp[2][10];
	double p1 = 0,p2 = 0, h = 1, result;
	int row = 0, col = 0, index = 0, len1, len2, sign = 1;
	if(value[0] == '-') 
	{
		index++;
		sign = -1;
	}
	while(value[index] != 0)
	{
		if(value[index] != '.')
		{
			temp[row][col] = value[index];
			col++;
		}
		else
		{
			row++;
			len1 = col;
			col = 0;
		}
		index++;
	}
	if(row == 0)
	{
		len1 = col;
		for(int i = len1 - 1; i >= 0; i--)
		{
			p1 += ((uint8_t)temp[0][i] - 48) * h;
			h *= 10;
		}
		p2 = 0;
	}
	else
	{
		for(int i = len1 - 1; i >= 0; i--)
		{
			p1 += ((uint8_t)temp[0][i] - 48) * h;
			h *= 10;
		}
		len2 = col;
		h = 0.1;
		for(int i = 0; i < len2; i++)
		{
			p2 += ((uint8_t)temp[1][i] - 48) * h;
			h *= 0.1;
		}
	}
	result = sign*(p1 + p2);
	return result;
}

/** @brief  : Check header is correct or not
**  @agr    : Input header
**  @retval : Return correct or not 
**/
Check_Status IsCorrectHeader(char *header)
{
	if(header[0] == '$' & header[1] == 'G' & header[2] == 'N' & header[3] == 'G' & header[4] == 'L' & header[5] == 'L')
	{
		return Check_OK;
	}
	else return Check_NOK;
}

/** @brief  : Convert value to char array
**  @agr    : Input value, result buffer
**  @retval : None 
**/
Check_Status IsDouble(double value)
{
	if ((value - (uint32_t)value) != (double)0)
	{
		return Check_OK;
	}
	else return Check_NOK;
}
/** @brief  : Convert value to char array
**  @agr    : Input value, result buffer
**  @retval : None 
**/
uint8_t ToChar(double value, uint8_t *pBuffer)
{
	uint32_t BefD;
	double AftD;
	uint8_t buffer[20], index = 0, temp1, len, strleng = 0;
	if(value < (double)0)
	{
		value = -value;
		pBuffer[0] = (uint8_t)'-';
		strleng++;
		index++;
	}
	BefD = (uint32_t)value;
	AftD = value - (double)BefD;
	if (BefD == 0) 
	{
		pBuffer[0] = (uint8_t)'0';
		len = 1;
		strleng++;
	}
	else
	{
		while(BefD != 0)
		{
			temp1 = BefD % 10;
			BefD /= 10;
			buffer[index] = temp1 + 48;
			index++;
		}
		len = index;
		strleng = index;
		// take the befD value
		for (int i = 0; i < len; i++)
		{
			index--;
			pBuffer[i] = buffer[index];
		}
	}
	if(IsDouble(AftD))
	{
		pBuffer[len] = (uint8_t)'.';
		len++;
		strleng++;
		for (int i = 0; i < 6; i++)
		{
			AftD *= 10;
			pBuffer[len + i] = (uint8_t)AftD + 48;
			AftD = AftD - (uint8_t)AftD;
			strleng++;
		}
	}
	return strleng;
}

/** @brief  : Is valid data
**  @agr    : Input message
**  @retval : None 
**/
Check_Status IsValidData(char *input)
{
	if(input[0] == 'A') return Check_OK;
	else if(input[0] == 'V') return Check_NOK;
	return Check_NOK;
}

/*-------------------- Stanley Function ------------------*/
/** @brief  : ToRPM
**  @agr    : Input velocity in m/s from panel
**  @retval : RPM value
**/
double ToRPM(double vel)
{
	double result;
	result = (vel / (2 * pi * Wheel_Radius)) * 60;
	return result;
}

/** @brief  : Calculate Path Angle
**  @agr    : Pathx, pathy, result path theta
**  @retval : None 
**/
void PathAngular(double *X, double *Y, double *result)
{
	int i = 0;
	while(i < NbOfWP - 1)
	{
		result[i] = atan2(Y[i + 1] - Y[i], X[i + 1] - X[i]);
		i++;
	}
	result[i] = 0;
}

/** @brief  : Pi to Pi
**  @agr    : input angle
**  @retval : double value
**/
double Pi_To_Pi(double angle)
{
	double result;
	if(angle > pi)
	{
		result = angle - 2 * pi;
	}
	else if (angle < -pi)
	{
		result = angle + 2 * pi;
	}
	else
	{
		result = angle;
	}
	return result;
}

/** @brief  : Controller using Stanley algorithm
**  @agr    : current pose of the robot and Pathx, Pathy
**  @retval : Steering angle
**/
double StanleyControl(double x, double y, double theta, double *Px, double *Py, double *PYaw, double v)
{                   /*   Current pose of the robot   */ /*  Path coordinate  */ /*  ThetaP  */
	double dmin = 0,dx,dy,d;
	int 	 index = 0;
	double efa, thetad, thetae, delta, tyaw, goal_radius;
	//Searching the nearest point
	for(int i = 0; i < NbOfWP; i++) 
	{
		dx = x - Px[i];
		dy = y - Py[i];
		d  = sqrt(pow(dx,2) + pow(dy,2));
		if(i == 0)
		{
			dmin 	= d;
			index = i;
		}
		else
		{
			if(dmin > d) // d is the new value that near the point
			{
				dmin 	= d;  // min 
				index = i;	// position of the minimum value
			}
		}
	}
	efa  = dmin;
	tyaw = Pi_To_Pi(atan2(y - Py[index],x - Px[index]) - theta);
	if(tyaw >= 0)
		efa = -efa;
	goal_radius = sqrt(pow(x - Px[NbOfWP - 1],2) + pow(y - Py[NbOfWP - 1],2));
	if(goal_radius <= 3)
		delta = 0;
	thetae = Pi_To_Pi(PYaw[index] - theta);
	thetad = atan2(K * efa,v);
	delta  = thetae + thetad;
	return delta;
}

/*-------------------- Fuzzy control -----------------------*/
/* Variables of fuzzy control block */
/** @brief  : Find maximum number
**  @agr    : 2 input values
**  @retval : Output maximum value
**/
double Fuzzy_Min(double in1, double in2)
{
	double min;
	min = in1;
	if(min > in2) min = in2;
	return min;
}

double Fuzzy_Max(double *input,int len)
{
	double max;
  max = input[0];
	for(int i = 1; i < len; i++)
	{
		if(max < input[i])
		{
			max = input[i];
		}
	}
	return max;
}

/** @brief  : Input 1 - error
**  @agr    : Input value, name of the symbols value
**  @retval : Output value
**/
double Input1(double x, char c[2])
{
	double result;
	if((c[0] == 'N') && (c[1] == 'B'))
	{
		if(x < In1_NB.h3)
			result = 1;
		else if(x < In1_NB.h4)
		{
			result = (In1_NB.h4 - x) / (In1_NB.h4 - In1_NB.h3);
		}
		else result = 0;
	}
	else if((c[0] == 'N') && (c[1] == 'S'))
	{
		if(x < In1_NS.a1)
			result = 0;
		else if(x < In1_NS.a2)
		{
			result = (x - In1_NS.a1) / (In1_NS.a2 - In1_NS.a1);
		}
		else if(x < In1_NS.a3)
		{
			result = (In1_NS.a3 - x) / (In1_NS.a3 - In1_NS.a2);
		}
		else result = 0;
	}
	else if((c[0] == 'Z') && (c[1] == 'E'))
	{
		if(x < In1_ZE.a1)
			result = 0;
		else if(x < In1_ZE.a2)
		{
			result = (x - In1_ZE.a1) / (In1_ZE.a2 - In1_ZE.a1);
		}
		else if(x < In1_ZE.a3)
		{
			result = (In1_ZE.a3 - x) / (In1_ZE.a3 - In1_ZE.a2);
		}
		else result = 0;
	}
	else if((c[0] == 'P') && (c[1] == 'S'))
	{
		if(x < In1_PS.a1)
			result = 0;
		else if(x < In1_PS.a2)
		{
			result = (x - In1_PS.a1) / (In1_PS.a2 - In1_PS.a1);
		}
		else if(x < In1_PS.a3)
		{
			result = (In1_PS.a3 - x) / (In1_PS.a3 - In1_PS.a2);
		}
		else result = 0;
	}
	else if((c[0] == 'P') && (c[1] == 'B'))
	{
		if(x < In1_PB.h1)
			result = 0;
		else if(x < In1_PB.h2)
		{
			result = (x - In1_PB.h1) / (In1_PB.h2 - In1_PB.h1);
		}
		else result = 1;
	}
	else result = 0;
	return result;
}
/** @brief  : Input 2 - velocity error
**  @agr    : Input value, name of the value
**  @retval : Output value
**/
double Input2(double x, char c[2])
{
	double result;
	if((c[0] == 'N') && (c[1] == 'E'))
	{
		if(x < In2_NE.h3)
			result = 1;
		else if(x < In2_NE.h4)
		{
			result = (In2_NE.h4 - x) / (In2_NE.h4 - In2_NE.h3);
		}
		else result = 0;
	}
	else if((c[0] == 'Z') && (c[1] == 'E'))
	{
		if(x < In2_ZE.a1)
			result = 0;
		else if(x < In2_ZE.a2)
		{
			result = (x - In2_ZE.a1) / (In2_ZE.a2 - In2_ZE.a1);
		}
		else if(x < In2_ZE.a3)
		{
			result = (In2_ZE.a3 - x) / (In2_ZE.a3 - In2_ZE.a2);
		}
		else result = 0;
	}
	else if((c[0] == 'P') && (c[1] == 'O'))
	{
		if(x < In2_PO.h1) 
			result = 0;
		else if(x < In2_PO.h2)
		{
			result = (x - In2_PO.h1) / (In2_PO.h2 - In2_PO.h1);
		}
		else result = 1;
	}
	else result = 0;
	return result;
}

/** @brief  : Defuzzification Max Min
**  @agr    : 2 input value
**  @retval : Output value
**/

double Defuzzification_Max_Min(double in1, double in2)
{
	double pBeta[5],num = 0, den = 0, temp;
	double result;
	//NB and NE is NB
	pBeta[0] = Fuzzy_Min(Input1(in1,"NB"),Input2(in2,"NE"));
	num = NB * pBeta[0];
	den = pBeta[0];
	//NS and NE is NM
	//NB and ZE is NM
	pBeta[0] = Fuzzy_Min(Input1(in1,"NS"),Input2(in2,"NE"));
	pBeta[1] = Fuzzy_Min(Input1(in1,"NB"),Input2(in2,"ZE"));
	temp = Fuzzy_Max(pBeta,2);
	num += NM * temp;
	den += temp;
	//ZE and NE is NS
	//NS and ZE is NS
	//NB and PO is NS
	pBeta[0] = Fuzzy_Min(Input1(in1,"ZE"),Input2(in2,"NE"));
	pBeta[1] = Fuzzy_Min(Input1(in1,"NS"),Input2(in2,"ZE"));
	pBeta[2] = Fuzzy_Min(Input1(in1,"NB"),Input2(in2,"PO"));
	temp = Fuzzy_Max(pBeta,3);
	num += NS * temp;
	den += temp;
	//PS and NE is ZE
	//ZE and ZE is ZE
	//NS and PO is ZE
	pBeta[0] = Fuzzy_Min(Input1(in1,"PS"),Input2(in2,"NE"));
	pBeta[1] = Fuzzy_Min(Input1(in1,"ZE"),Input2(in2,"ZE"));
	pBeta[2] = Fuzzy_Min(Input1(in1,"NS"),Input2(in2,"PO"));
	temp = Fuzzy_Max(pBeta,3);
	num += ZE * temp;
	den += temp;
	//PB and NE is PS
	//PS and ZE is PS
	//ZE and PO is PS
	pBeta[0] = Fuzzy_Min(Input1(in1,"PB"),Input2(in2,"NE"));
	pBeta[1] = Fuzzy_Min(Input1(in1,"PS"),Input2(in2,"ZE"));
	pBeta[2] = Fuzzy_Min(Input1(in1,"ZE"),Input2(in2,"PO"));
	temp = Fuzzy_Max(pBeta,3);
	num += PS * temp;
	den += temp;
	//PB and ZE is PM
	//PS and PO is PM
	pBeta[0] = Fuzzy_Min(Input1(in1,"PB"),Input2(in2,"ZE"));
	pBeta[1] = Fuzzy_Min(Input1(in1,"PS"),Input2(in2,"PO"));
	temp = Fuzzy_Max(pBeta,2);
	num += PM * temp;
	den += temp;
	//PB and PO is PB
	pBeta[0] = Fuzzy_Min(Input1(in1,"PB"),Input2(in2,"PO"));
	num += PB * pBeta[0];
	den += pBeta[0];
	result = num / den;
	return result;
}

/** @brief  : Defuzzification Max Prod
**  @agr    : Input value, name of the symbols value
**  @retval : Output value
**/
double Defuzzification_Max_Prod(void)
{
	double result;
	return result;
}



















