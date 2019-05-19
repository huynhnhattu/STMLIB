#include "TwoWheelRobot.h"

SelfPosition selfPosition;
SelfPosition *pt;


void Self_ParametersInit(SelfPosition *pself)
{
	pself->L  = 37.3;
	pself->R  = 8.3;
	pself->x  = 0;
	pself->y  = 0;
	pself->theta = 0;
}

void SelfPositionUpdateParams(SelfPosition *selfPos, double w_left, double w_right, double sampleTime)
{
    double v_left, v_right, v_dir, w_dir;
    // Calculate velocity with multiple directions
    v_left = selfPos->R * w_left * sampleTime;
    v_right = selfPos->R * w_right * sampleTime;
    w_dir = (v_right - v_left) / (2 * selfPos->L);
    v_dir = (v_right + v_left) / 2;
    //selfPos->theta = selfPos->theta + w_dir;

    // Update new position
    selfPos->x = selfPos->x + v_dir * cos(selfPos->theta);
    selfPos->y = selfPos->y + v_dir * sin(selfPos->theta);
}

void OverWritePosition(SelfPosition *selfPos, double x, double y)
{
    selfPos->x = x;
    selfPos->y = y;
}




