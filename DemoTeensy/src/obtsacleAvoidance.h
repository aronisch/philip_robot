#ifndef _OBSTACLE_AVOIDANCE_H_
#define _OBSTACLE_AVOIDANCE_H_
#include <Arduino.h>
#include "MotorControl/DifferentialDrive.h"
#include "MotorControl/Odometry.h"
#include "sensors/lidar.h"

#define LINEAR_SPEED (200)
#define ANGULAR_SPEED (3.14159f)

#define STOPPING_TIME (200)
#define TURNING_TIME (HALF_PI/ANGULAR_SPEEDf)

#define AVOIDING_OBJECT_DISTANCE (300)

void avoidingObstacle(DifferentialDrive &diffD, Odometry &odo, Lidar &sensor);

#endif 