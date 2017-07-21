#include "mpu6050_process.h"
#include "filter.h"

//校准时间
#define ACC_CALC_TIME  3000//ms
#define GYRO_CALC_TIME   3000000l	//us

/*
 *   MPU6050_Filter
 *   该函数用于滤波MPU6050得到的原始数据
 *
 *   参数：
 *     MPU6050_DATA
 *        |__ MPU6050原始数据
 *     MPU6050_FILTED
 *        |__ MPU6050滤波之后得到的数据
 *     MPU6050_OFFSET
 *        |__ MPU6050通过开机检测，得到的初始位置的偏移量
 *   返回值
 *      无
 */
#define SENSOR_MAX_G 8.0f		//constant g		// tobe fixed to 8g. but IMU need to  correct at the same time
#define SENSOR_MAX_W 2000.0f	//deg/s
#define ACC_SCALE  (SENSOR_MAX_G/32768.0f)
#define GYRO_SCALE  (SENSOR_MAX_W/32768.0f)

void MPU6050_Filter(MPU6050_DATA_STRUCT* MPU6050_DATA,
                MPU6050_PHYSICAL_STRUCT* MPU6050_FILTED,
                MPU6050_PHYSICAL_STRUCT* MPU6050_OFFSET)
{
        //    uint8_t i;
        MPU6050_PHYSICAL_STRUCT MPU6050_RAW;
        //read raw
        //raw data saved in MPU6050_DATA
        //turn to physical
        MPU6050_RAW.acc_x = (float)MPU6050_DATA->acc_x * ACC_SCALE * CONSTANTS_ONE_G ;
        MPU6050_RAW.acc_y = (float)MPU6050_DATA->acc_y * ACC_SCALE * CONSTANTS_ONE_G ;
        MPU6050_RAW.acc_z = (float)MPU6050_DATA->acc_z * ACC_SCALE * CONSTANTS_ONE_G ;

        MPU6050_RAW.gyr_x = (float)MPU6050_DATA->gyr_x * GYRO_SCALE * M_PI_F /180.f;		//deg/s
        MPU6050_RAW.gyr_y = (float)MPU6050_DATA->gyr_y * GYRO_SCALE * M_PI_F /180.f;		//deg/s
        MPU6050_RAW.gyr_z = (float)MPU6050_DATA->gyr_z * GYRO_SCALE * M_PI_F /180.f;		//deg/s

        //filter
        MPU6050_FILTED->acc_x = LPF2pApply_1(MPU6050_RAW.acc_x-MPU6050_OFFSET->acc_x);
        MPU6050_FILTED->acc_y = LPF2pApply_2(MPU6050_RAW.acc_y-MPU6050_OFFSET->acc_y);
        MPU6050_FILTED->acc_z = LPF2pApply_3(MPU6050_RAW.acc_z-MPU6050_OFFSET->acc_z);

        MPU6050_FILTED->gyr_x = LPF2pApply_4(MPU6050_RAW.gyr_x);
        MPU6050_FILTED->gyr_y = LPF2pApply_5(MPU6050_RAW.gyr_y);
        MPU6050_FILTED->gyr_z = LPF2pApply_6(MPU6050_RAW.gyr_z);
}


//! Auxiliary variables to reduce number of repeated operations
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
/** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;
/** quaternion of sensor frame relative to auxiliary frame */
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;
static uint8_t bFilterInit = 0;
//static uint8_t bImuReady = 0;

//函数名：invSqrt(void)
//描述：求平方根的倒数
//该函数是经典的Carmack求平方根算法，效率极高，使用魔数0x5f375a86
static float invSqrt(float number)
{
        volatile long i;
        volatile float x, y;
        volatile const float f = 1.5F;

        x = number * 0.5F;
        y = number;
        i = * (( long * ) &y);
        i = 0x5f375a86 - ( i >> 1 );
        y = * (( float * ) &i);
        y = y * ( f - ( x * y * y ) );
        return y;
}

/*
 *   NonlinearSO3AHRSinit
 *   该函数用于包含磁力计的MPU6050初始化，得到最初的表示姿态的四元数
 *
 *   参数：
 *     ax, ay, az, mx, my, mz
 *        |__ MPU6050滤波后的数据
 *   返回值
 *      无
 */
//! Using accelerometer, sense the gravity vector.
//! Using magnetometer, sense yaw.
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
        float initialRoll, initialPitch;
        float cosRoll, sinRoll, cosPitch, sinPitch;
        float magX, magY;
        float initialHdg, cosHeading, sinHeading;

        initialRoll = atan2(-ay, -az);
        initialPitch = atan2(ax, -az);

        cosRoll = cosf(initialRoll);
        sinRoll = sinf(initialRoll);
        cosPitch = cosf(initialPitch);
        sinPitch = sinf(initialPitch);

        magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

        magY = my * cosRoll - mz * sinRoll;

        initialHdg = atan2f(-magY, magX);

        cosRoll = cosf(initialRoll * 0.5f);
        sinRoll = sinf(initialRoll * 0.5f);

        cosPitch = cosf(initialPitch * 0.5f);
        sinPitch = sinf(initialPitch * 0.5f);

        cosHeading = cosf(initialHdg * 0.5f);
        sinHeading = sinf(initialHdg * 0.5f);

        q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
        q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
        q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
        q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

        // auxillary variables to reduce number of repeated operations, for 1st pass
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;
}
/*
 *   函数名：NonlinearSO3AHRSupdate
 *   描述：姿态解算第一次调用初始化，之后每次更新四元数的值
 *   使用的是Mahony互补滤波算法，没有使用Kalman滤波算法
 *   该算法是直接参考pixhawk飞控的算法，可以在Github上看到出处
 *   https://github.com/hsteinhaus/PX4Firmware/blob/master/src/modules/attitude_estimator_so3/attitude_estimator_so3_main.cpp
 *   参数：
 *     gx, gy, gz, ax, ay, az, mx, my, mz
 *        |__ MPU6050滤波后的数据
 *     twoKp, twoKi
 *        |__ 进行PI控制的参数
 *     dt
 *        |__ MPU6050刷新频率，由中断时间决定，在这里是10ms
 *   返回值
 *      无
 */
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float twoKp, float twoKi, float dt)
{
        float recipNorm;
        float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

        // Make filter converge to initial solution faster
        // This function assumes you are in static position.
        // WARNING : in case air reboot, this can cause problem. But this is very unlikely happen.
        if(bFilterInit == 0) {
                NonlinearSO3AHRSinit(ax,ay,az,mx,my,mz);
                bFilterInit = 1;
        }

        //! If magnetometer measurement is available, use it.
        //仅当磁力计可用时，进入该语句块，【不可用时将磁力计置0】
        if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
                float hx, hy, hz, bx, bz;
                float halfwx, halfwy, halfwz;

                // Normalise magnetometer measurement
                // Will sqrt work better? PX4 system is powerful enough?
                recipNorm = invSqrt(mx * mx + my * my + mz * mz);
                mx *= recipNorm;
                my *= recipNorm;
                mz *= recipNorm;

                // Reference direction of Earth's magnetic field
                hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
                hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
                hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
                bx = sqrt(hx * hx + hy * hy);
                bz = hz;

                // Estimated direction of magnetic field
                halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
                halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
                halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

                // Error is sum of cross product between estimated direction and measured direction of field vectors
                halfex += (my * halfwz - mz * halfwy);
                halfey += (mz * halfwx - mx * halfwz);
                halfez += (mx * halfwy - my * halfwx);
        }

        //增加一个条件：  加速度的模量与G相差不远时。 0.75*G < normAcc < 1.25*G
        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
        {
                float halfvx, halfvy, halfvz;

                // Normalise accelerometer measurement
                //归一化，得到单位加速度
                recipNorm = invSqrt(ax * ax + ay * ay + az * az);
                ax *= recipNorm;
                ay *= recipNorm;
                az *= recipNorm;

                // Estimated direction of gravity and magnetic field
                halfvx = q1q3 - q0q2;
                halfvy = q0q1 + q2q3;
                halfvz = q0q0 - 0.5f + q3q3;

                // Error is sum of cross product between estimated direction and measured direction of field vectors
                halfex += ay * halfvz - az * halfvy;
                halfey += az * halfvx - ax * halfvz;
                halfez += ax * halfvy - ay * halfvx;
        }

        // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
        if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
                // Compute and apply integral feedback if enabled
                if(twoKi > 0.0f) {
                        gyro_bias[0] += twoKi * halfex * dt;	// integral error scaled by Ki
                        gyro_bias[1] += twoKi * halfey * dt;
                        gyro_bias[2] += twoKi * halfez * dt;

                        // apply integral feedback
                        gx += gyro_bias[0];
                        gy += gyro_bias[1];
                        gz += gyro_bias[2];
                }
                else {
                        gyro_bias[0] = 0.0f;	// prevent integral windup
                        gyro_bias[1] = 0.0f;
                        gyro_bias[2] = 0.0f;
                }

                // Apply proportional feedback
                gx += twoKp * halfex;
                gy += twoKp * halfey;
                gz += twoKp * halfez;
        }

        //! Integrate rate of change of quaternion
#if 0
        gx *= (0.5f * dt);		// pre-multiply common factors
        gy *= (0.5f * dt);
        gz *= (0.5f * dt);
#endif

        // Time derivative of quaternion. 
	//q_dot = 0.5*q\otimes \omega.
        //! q_k = q_{k-1} + dt*\dot{q}
        //! \dot{q} = 0.5*q \otimes P(\omega)
        dq0 = 0.5f*(-q1 * gx - q2 * gy - q3 * gz);
        dq1 = 0.5f*(q0 * gx + q2 * gz - q3 * gy);
        dq2 = 0.5f*(q0 * gy - q1 * gz + q3 * gx);
        dq3 = 0.5f*(q0 * gz + q1 * gy - q2 * gx);

        q0 += dt*dq0;
        q1 += dt*dq1;
        q2 += dt*dq2;
        q3 += dt*dq3;

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;
}


#define so3_comp_params_Kp 1.0f
#define so3_comp_params_Ki  0.05f

//函数名：MPU6050_Process(MPU6050_DATA_STRUCT*, MPU6050_EULER_STRUCT*)
//描述：姿态软件解算融合函数
//该函数对姿态的融合是软件解算
//[1]原始值滤波，并由滤波函数把AD值转换为物理量
//[2]更新角速度偏移量，避免积分误差
//[3]姿态融合（调用姿态融合函数），产生四元数
//[4]把四元数转换成旋转矩阵和欧拉角，分别输出

void MPU6050_Process(MPU6050_DATA_STRUCT* MPU6050_DATA,
                MPU6050_PHYSICAL_STRUCT* MPU6050_FILTED,
                MPU6050_PHYSICAL_STRUCT* MPU6050_OFFSET,
                MPU6050_EULER_STRUCT* MPU6050_EULER_Rad,
                MPU6050_EULER_STRUCT* MPU6050_EULER)
{
        //! Time constant
        float dt = 0.01f;		//s
        //    static uint32_t tPrev=0,startTime=0;	//us
        //    uint32_t now;
        //    uint8_t i;

        /* output euler angles */
        float euler[3] = {0.0f, 0.0f, 0.0f};	//rad

        /* Initialization */
        float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };		/**< init: identity matrix */
        float acc[3] = {0.0f, 0.0f, 0.0f};		//m/s^2
        float gyro[3] = {0.0f, 0.0f, 0.0f};		//rad/s
        float mag[3] = {0.0f, 0.0f, 0.0f};
        //need to calc gyro offset before imu start working
        //    static float gyro_offsets_sum[3]= { 0.0f, 0.0f, 0.0f }; // gyro_offsets[3] = { 0.0f, 0.0f, 0.0f },
        //    static uint16_t offset_count = 0;

        //    now=micros();
        //    dt=(tPrev>0)?(now-tPrev)/1000000.0f:0;
        dt=10000.0f/1000000.0f;
        //    tPrev=now;

        MPU6050_Filter(MPU6050_DATA, MPU6050_FILTED, MPU6050_OFFSET);
        //  if(!bImuReady)
        //  {
        //      if(startTime==0)
        //          startTime=now;
        //      //通过对滤波值进行累加，得到很长时间的平均角速度
        //      gyro_offsets_sum[0] += MPU6050_FILTED.gyr_x;
        //      gyro_offsets_sum[1] += MPU6050_FILTED.gyr_y;
        //      gyro_offsets_sum[2] += MPU6050_FILTED.gyr_z;
        //      offset_count++;
        //
        //      //每隔一段时间更新一次角速度的偏移量，以免出现积分误差
        //      //通过累积一段时间的角速度的量获得
        //      if(now > startTime + GYRO_CALC_TIME)
        //      {
        //          MPU6050_OFFSET.gyr_x = gyro_offsets_sum[0]/offset_count;
        //          MPU6050_OFFSET.gyr_y = gyro_offsets_sum[1]/offset_count;
        //          MPU6050_OFFSET.gyr_z = gyro_offsets_sum[2]/offset_count;
        //
        //          offset_count=0;
        //          gyro_offsets_sum[0]=0;
        //          gyro_offsets_sum[1]=0;
        //          gyro_offsets_sum[2]=0;
        //
        //          bImuReady = 1;
        //          startTime=0;
        //
        //      }
        //      return;
        //  }


        acc[0] = -MPU6050_FILTED->acc_x;
        acc[1] = -MPU6050_FILTED->acc_y;
        acc[2] = -MPU6050_FILTED->acc_z;

        gyro[0] = MPU6050_FILTED->gyr_x - MPU6050_OFFSET->gyr_x;
        gyro[1] = MPU6050_FILTED->gyr_y - MPU6050_OFFSET->gyr_y;
        gyro[2] = MPU6050_FILTED->gyr_z - MPU6050_OFFSET->gyr_z;

        // NOTE : Accelerometer is reversed.
        // Because proper mount of PX4 will give you a reversed accelerometer readings.
        NonlinearSO3AHRSupdate(gyro[0], gyro[1], gyro[2],
                               -acc[0], -acc[1], -acc[2],
                                mag[0],  mag[1],  mag[2],
                               so3_comp_params_Kp,
                               so3_comp_params_Ki,
                               dt);

        // Convert q->R, This R converts inertial frame to body frame.
        Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;  // 11
        Rot_matrix[1] = 2.f * (q1*q2 + q0*q3);	// 12
        Rot_matrix[2] = 2.f * (q1*q3 - q0*q2);	// 13
        Rot_matrix[3] = 2.f * (q1*q2 - q0*q3);	// 21
        Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;  // 22
        Rot_matrix[5] = 2.f * (q2*q3 + q0*q1);	// 23
        Rot_matrix[6] = 2.f * (q1*q3 + q0*q2);	// 31
        Rot_matrix[7] = 2.f * (q2*q3 - q0*q1);	// 32
        Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;  // 33

        //1-2-3 Representation.
        //Equation (290)
        //Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
        // Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
        euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);	//! Roll
        euler[1] = -asinf(Rot_matrix[2]);			//! Pitch
        euler[2] = atan2f(Rot_matrix[1], Rot_matrix[0]);	//! Yaw

        //DCM . ground to body
        //    for(i=0; i<9; i++)
        //    {
        //        *(&(imu.DCMgb[0][0]) + i)=Rot_matrix[i];
        //    }

	




        MPU6050_EULER_Rad->roll=euler[0];
        MPU6050_EULER_Rad->pitch=euler[1];
        MPU6050_EULER_Rad->yaw=euler[2];

        MPU6050_EULER->roll=euler[0] * 180.0f / M_PI_F;
        MPU6050_EULER->pitch=euler[1] * 180.0f / M_PI_F;
        MPU6050_EULER->yaw=euler[2] * 180.0f / M_PI_F;
}


void InitOffset6050()
{
  const uint32_t T_now = T;
  while(T - T_now < ACC_CALC_TIME)
  {
  
  }
    
  MPU6050_OFFSET.gyr_x = gyro_offsets_sum[0]/offset_count;
  MPU6050_OFFSET.gyr_y = gyro_offsets_sum[1]/offset_count;
  MPU6050_OFFSET.gyr_z = gyro_offsets_sum[2]/offset_count;
  
}