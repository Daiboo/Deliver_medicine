
#include <stdint.h>

#define GYRO_CALIBRATION_COFF 0.0152672f;  //500


// acceleration due to gravity in m/s/s
#define GRAVITY_MSS 9.80665f
#define GRAVITY_RAW 8192.0f


#ifndef M_PI_F
 #define M_PI_F 3.141592653589793f
#endif

#ifndef PI
 # define PI M_PI_F
#endif

#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

float constrain_float(float amt, float low, float high);
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);

void FastSinCos(float x, float *sinVal, float *cosVal);
float FastSin(float x);
float FastCos(float x);
float sq(float v);
float FastSqrt(float x);
float safe_sqrt(float v);
#define sq2(sq) (((float)sq) * ((float)sq))

float invSqrt(float x);



