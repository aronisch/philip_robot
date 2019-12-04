#include "obtsacleAvoidance.h"

bool avoidingObstacle(DifferentialDrive &diffD, Odometry &odo, Lidar &sensor){
    static enum {STOP, TURN_LEFT1, MOVE_AWAY, TURN_RIGHT1, MOVE_FORWARD, TURN_RIGHT2, RETURN, TURN_LEFT2, NOT_AVOIDING} state;
    static uint32_t lastcall = millis();
    static bool actionDone = false;
    static bool hasSeenHigh = false;
    
    switch(state)
    {
        case STOP:
            if(!actionDone){
                lastcall = millis();
            }
            
            diffD.setVelocities(0.0, 0.0);
            actionDone = true;
            if ((millis() - lastcall) > STOPPING_TIME){
                state = TURN_LEFT1;
                odo.resetPosition();
                actionDone = false;
            }
            return false;
            break;
        
        case TURN_LEFT1:
            if(!actionDone){
                lastcall = millis();
            }
            actionDone = true;
            diffD.setVelocities(0.0, ANGULAR_SPEED);
            if ((millis() - lastcall) > TURNING_TIME){
                state = MOVE_AWAY;
                actionDone = false;
            }
            return false;
            break;
        
        case MOVE_AWAY:
            diffD.setVelocities(200, 0.0);
            
            if(sensor.getDist() < AVOIDING_OBJECT_DISTANCE){
                hasSeenHigh = true;
            }
            if(hasSeenHigh){
                if(sensor.getDist() > AVOIDING_OBJECT_DISTANCE){
                    state = TURN_RIGHT1;
                    actionDone = false;
                    hasSeenHigh = false;
                }
            }
            diffD.setVelocities(0.0, ANGULAR_SPEED);

            return false;
        
        case TURN_RIGHT1:
            if(!actionDone){
                lastcall = millis();
            }
            actionDone = true;
            diffD.setVelocities(0.0, -ANGULAR_SPEED);
            if ((millis() - lastcall) > TURNING_TIME){
                state = MOVE_FORWARD;
                actionDone = false;
            }
            return false;
            break;
        
        case MOVE_FORWARD:
            diffD.setVelocities(200, 0.0);
            
            if(sensor.getDist() < AVOIDING_OBJECT_DISTANCE){
                hasSeenHigh = true;
            }
            if(hasSeenHigh){
                if(sensor.getDist() > AVOIDING_OBJECT_DISTANCE){
                    state = TURN_RIGHT2;
                    hasSeenHigh = false;
                }
            }
            return false;
        
        case TURN_RIGHT2:
            if(!actionDone){
                lastcall = millis();
            }
            actionDone = true;
            diffD.setVelocities(0.0, -ANGULAR_SPEED);
            if ((millis() - lastcall) > TURNING_TIME){
                state = RETURN;
                actionDone = false;
            }
            return false;
            break;

        case RETURN:
            diffD.setVelocities(LINEAR_SPEED, 0.0);

            if(odo.getPositionY() <= 0){
                state = TURN_LEFT2;
            }
            return false;
            break;
        
        case TURN_LEFT2:
            if(!actionDone){
                lastcall = millis();
            }
            actionDone = true;
            diffD.setVelocities(0.0, ANGULAR_SPEED);
            if ((millis() - lastcall) > TURNING_TIME){
                state = NOT_AVOIDING;
                actionDone = false;
                return true;
            }
            return false;
            break;
        default:
            return true;
            break;
    }

    return true;
}