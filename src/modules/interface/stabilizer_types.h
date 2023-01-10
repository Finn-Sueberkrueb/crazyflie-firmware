/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * stabilizer.h: Stabilizer orchestrator
 */
#ifndef __STABILIZER_TYPES_H__
#define __STABILIZER_TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include "imu_types.h"
#include "lighthouse_types.h"

/* Data structure used by the stabilizer subsystem.
 * All have a timestamp to be set when the data is calculated.
 */

// TODO: avoid Structure Padding. The struct is send without Padding. During the cast the struct needs to be converted correctly for better Performence.
#pragma pack(push,1)
typedef struct {
  uint64_t timestamp;
  uint16_t motor_1;
  uint16_t motor_2;
  uint16_t motor_3;
  uint16_t motor_4;
  uint16_t frame;
  uint8_t status;
  //uint8_t empty1; // Padding

} t_extrenalActuator;


// 101 byte ???
typedef struct {
  uint64_t timestamp;
  float pos_x; // m
  float pos_y; // m
  float pos_z; // m
  float vel_x; // m/s
  float vel_y; // m/s
  float vel_z; // m/s
  float acc_x; // m/ss
  float acc_y; // m/ss
  float acc_z; // m/ss
  float q_1;
  //float q_2;
  //float q_3;
  //float q_4;
  float rot_x; // deg
  float rot_y; // deg
  float rot_z; // deg
  float rot_vel_x; // deg/s
  float rot_vel_y; // deg/s
  float rot_vel_z; // deg/s
  float rot_acc_x; // deg/ss
  float rot_acc_y; // deg/ss
  float rot_acc_z; // deg/ss
  float latency; //from sensor to actuator
  uint16_t motor_1;
  uint16_t motor_2;
  uint16_t motor_3;
  uint16_t motor_4;
  uint16_t frame;
  uint16_t lased_actuator_frame;
  uint8_t status;
  //uint8_t empty1; // Padding
  //uint16_t empty2; // Padding
  
} t_externalState;

#pragma pack(pop)

/** Attitude in euler angle form */
typedef struct attitude_s {
  uint32_t timestamp;  // Timestamp when the data was computed

  float roll;
  float pitch;
  float yaw;
} attitude_t;

/* vector */
#define vec3d_size 3
typedef float vec3d[vec3d_size];
typedef float mat3d[vec3d_size][vec3d_size];

/* x,y,z vector */
struct vec3_s {
  uint32_t timestamp; // Timestamp when the data was computed

  float x;
  float y;
  float z;
};

typedef struct vec3_s vector_t;
typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

/* Orientation as a quaternion */
typedef struct quaternion_s {
  uint32_t timestamp;

  union {
    struct {
      float q0;
      float q1;
      float q2;
      float q3;
    };
    struct {
      float x;
      float y;
      float z;
      float w;
    };
  };
} quaternion_t;

typedef enum {
  MeasurementSourceLocationService  = 0,
  MeasurementSourceLighthouse       = 1,
} measurementSource_t;

typedef struct tdoaMeasurement_s {
  point_t anchorPositions[2];
  uint8_t anchorIds[2];
  float distanceDiff;
  float stdDev;
} tdoaMeasurement_t;

typedef struct baro_s {
  float pressure;           // mbar
  float temperature;        // degree Celcius
  float asl;                // m (ASL = altitude above sea level)
} baro_t;

typedef struct positionMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  float stdDev;
  measurementSource_t source;
} positionMeasurement_t;

typedef struct poseMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  quaternion_t quat;
  float stdDevPos;
  float stdDevQuat;
} poseMeasurement_t;

typedef struct distanceMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  uint8_t anchorId;
  float distance;
  float stdDev;
} distanceMeasurement_t;

typedef struct zDistance_s {
  uint32_t timestamp;
  float distance;           // m
} zDistance_t;

typedef struct sensorData_s {
  Axis3f acc;               // Gs
  Axis3f gyro;              // deg/s
  Axis3f mag;               // gauss
  baro_t baro;
#ifdef LOG_SEC_IMU
  Axis3f accSec;            // Gs
  Axis3f gyroSec;           // deg/s
#endif
  uint64_t interruptTimestamp;
} sensorData_t;

typedef struct state_s {
  attitude_t attitude;      // deg (legacy CF2 body coordinate system, where pitch is inverted)
  quaternion_t attitudeQuaternion;
  point_t position;         // m
  velocity_t velocity;      // m/s
  acc_t acc;                // Gs (but acc.z without considering gravity)
} state_t;

#define STABILIZER_NR_OF_MOTORS 4

typedef enum control_mode_e {
  controlModeLegacy      = 0, // legacy mode with int16_t roll, pitch, yaw and float thrust
  controlModeForceTorque = 1,
  controlModeForce       = 2,
} control_mode_t;

typedef struct control_s {
  union {
    // controlModeLegacy
    struct {
      int16_t roll;
      int16_t pitch;
      int16_t yaw;
      float thrust;
    };

    // controlModeForceTorque
    // Note: Using SI units for a controller makes it hard to tune it for different platforms. The normalized force API
    // is probably a better option.
    struct {
      float thrustSi;  // N
      union { // Nm
        float torque[3];
        struct {
          float torqueX;
          float torqueY;
          float torqueZ;
        };
      };
    };

    // controlModeForce
    float normalizedForces[STABILIZER_NR_OF_MOTORS]; // 0.0 ... 1.0
  };

  control_mode_t controlMode;
} control_t;

typedef union {
  int32_t list[STABILIZER_NR_OF_MOTORS];
  struct {
    int32_t m1;
    int32_t m2;
    int32_t m3;
    int32_t m4;
  } motors;
} motors_thrust_uncapped_t;

typedef union {
  uint16_t list[STABILIZER_NR_OF_MOTORS];
  struct {
    uint16_t m1;  // PWM ratio
    uint16_t m2;  // PWM ratio
    uint16_t m3;  // PWM ratio
    uint16_t m4;  // PWM ratio
  } motors;
} motors_thrust_pwm_t;

typedef enum mode_e {
  modeDisable = 0,
  modeAbs,
  modeVelocity
} stab_mode_t;

typedef struct setpoint_s {
  uint32_t timestamp;

  attitude_t attitude;      // deg
  attitude_t attitudeRate;  // deg/s
  quaternion_t attitudeQuaternion;
  float thrust;
  point_t position;         // m
  velocity_t velocity;      // m/s
  acc_t acceleration;       // m/s^2
  bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame

  struct {
    stab_mode_t x;
    stab_mode_t y;
    stab_mode_t z;
    stab_mode_t roll;
    stab_mode_t pitch;
    stab_mode_t yaw;
    stab_mode_t quat;
  } mode;
} setpoint_t;

/** Estimate of position */
typedef struct estimate_s {
  uint32_t timestamp; // Timestamp when the data was computed

  point_t position;
} estimate_t;

/** Setpoint for althold */
typedef struct setpointZ_s {
  float z;
  bool isUpdate; // True = small update of setpoint, false = completely new
} setpointZ_t;

/** Flow measurement**/
typedef struct flowMeasurement_s {
  uint32_t timestamp;
  union {
    struct {
      float dpixelx;  // Accumulated pixel count x
      float dpixely;  // Accumulated pixel count y
    };
    float dpixel[2];  // Accumulated pixel count
  };
  float stdDevX;      // Measurement standard deviation
  float stdDevY;      // Measurement standard deviation
  float dt;           // Time during which pixels were accumulated
} flowMeasurement_t;


/** TOF measurement**/
typedef struct tofMeasurement_s {
  uint32_t timestamp;
  float distance;
  float stdDev;
} tofMeasurement_t;

/** Absolute height measurement */
typedef struct heightMeasurement_s {
  uint32_t timestamp;
  float height;
  float stdDev;
} heightMeasurement_t;

/** Yaw error measurement */
typedef struct {
  uint32_t timestamp;
  float yawError;
  float stdDev;
} yawErrorMeasurement_t;

/** Sweep angle measurement */
typedef struct {
  uint32_t timestamp;
  const vec3d* sensorPos;    // Sensor position in the CF reference frame
  const vec3d* rotorPos;     // Pos of rotor origin in global reference frame
  const mat3d* rotorRot;     // Rotor rotation matrix
  const mat3d* rotorRotInv;  // Inverted rotor rotation matrix
  uint8_t sensorId;
  uint8_t baseStationId;
  uint8_t sweepId;
  float t;                   // t is the tilt angle of the light plane on the rotor
  float measuredSweepAngle;
  float stdDev;
  const lighthouseCalibrationSweep_t* calib;
  lighthouseCalibrationMeasurementModel_t calibrationMeasurementModel;
} sweepAngleMeasurement_t;

/** gyroscope measurement */
typedef struct
{
  Axis3f gyro; // deg/s, for legacy reasons
} gyroscopeMeasurement_t;

/** accelerometer measurement */
typedef struct
{
  Axis3f acc; // Gs, for legacy reasons
} accelerationMeasurement_t;

/** barometer measurement */
typedef struct
{
  baro_t baro; // for legacy reasons
} barometerMeasurement_t;


// Frequencies to bo used with the RATE_DO_EXECUTE_HZ macro. Do NOT use an arbitrary number.
#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_250_HZ 250
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25

#define RATE_MAIN_LOOP RATE_1000_HZ
#define ATTITUDE_RATE RATE_500_HZ
#define POSITION_RATE RATE_100_HZ
#define RATE_HL_COMMANDER RATE_100_HZ

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)

#endif
