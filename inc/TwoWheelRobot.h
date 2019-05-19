#include <math.h>

typedef struct SelfPosition
{
    double x;
    double y;
    double w_left;
    double w_right;
    double R;
    double L;
    double theta;
} SelfPosition;

void Self_ParametersInit(SelfPosition *pself);
void SelfPositionUpdateParams(SelfPosition *selfPos, double w_left, double w_right, double sampleTime);
void OverWritePosition(SelfPosition *selfPos, double x, double y);

extern SelfPosition selfPosition;



