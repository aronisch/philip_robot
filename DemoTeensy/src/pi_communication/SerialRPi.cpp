#include "SerialRPi.h"

void SerialRPi::update(){
    while(_serialPort->available()){
        switch(_serialPort->read()){
            //Velocities Settings
            case 'L':
            {
                int linearSpeed = _serialPort->parseInt();
                int angularSpeed = 0;
                if(_serialPort->available()){
                    if(_serialPort->read() == 'A'){
                        angularSpeed = _serialPort->parseInt();
                        _diffDrive->setVelocities((double) linearSpeed, (double) angularSpeed*2*M_PI/360);
                        Serial.print("Linear Speed:");
                        Serial.print(linearSpeed);
                        Serial.print("Angular Speed:");
                        Serial.println(angularSpeed);
                    }
                }
                break;
            }
            //Plate holder position
            case 'P':
            {
                int cmd = _serialPort->read();
                if(cmd == '0'){
                    _plateHolder->close();
                    Serial.println("Close Plate");
                } else if(cmd == '1'){
                    _plateHolder->open();
                    Serial.println("Open Plate");
                }
                break;
            }
            //Gripper position
            case 'G':
            {
                int cmd = _serialPort->read();
                if(cmd == '0'){
                    _gripperArm->close();
                    Serial.println("Close Gripper");
                } else if(cmd == '1'){
                    _gripperArm->open();
                    Serial.println("Open Gripper");
                }
                break;
            }
            //Gripper arm position
            case 'R':
            {
                int cmd = _serialPort->read();
                if(cmd == '0'){
                    _gripperArm->toFront();
                    Serial.println("Arm Front");
                } else if(cmd == '1'){
                    _gripperArm->toPlate();
                    Serial.println("Arm Plate");
                }
                break;
            }
            //Check if obstacle avoidance activated
            case 'O':
            {
                //
                break;
            }

            //Return Odometry
            case 'D':
            {
                //_serialPort->print("X");
                _serialPort->println((int)_odo->getPositionX());
                //_serialPort->print(" Y");
                _serialPort->println((int)_odo->getPositionY());
                //_serialPort->print(" T");
                _serialPort->println((int)(_odo->getPositionTheta()/M_PI*180));
                //_serialPort->println();
                break;
            }

            //Reset Odometry
            case 'N':
            {
                _odo->resetPosition();
                break;
            }
        }
    }
}