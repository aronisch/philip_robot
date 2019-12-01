#include "SerialRPi.h"

void SerialRPi::update(){
    while(_serialPort->available()){
        switch(_serialPort->read()){
            //Velocities Settings
            case 'L':
                int linearSpeed = _serialPort->parseInt();
                int angularSpeed = 0;
                if(_serialPort->available()){
                    if(_serialPort->read() == 'A'){
                        angularSpeed = _serialPort->parseInt();
                        _diffDrive->setVelocities((double) linearSpeed, (double) angularSpeed);
                    }
                }
                break;

            //Plate holder position
            case 'P':
                int cmd = _serialPort->read();
                if(cmd == '0'){
                    _plateHolder->close();
                } else if(cmd == '1'){
                    _plateHolder->open();
                }
                break;

            //Gripper position
            case 'G':
                int cmd = _serialPort->read();
                if(cmd == '0'){
                    _gripperArm->close();
                } else if(cmd == '1'){
                    _gripperArm->open();
                }
                break;

            //Gripper arm position
            case 'R':
                int cmd = _serialPort->read();
                if(cmd == '0'){
                    _gripperArm->toFront();
                } else if(cmd == '1'){
                    _gripperArm->toPlate();
                }
                break;

            //Check if obstacle avoidance activated
            case 'O':
                //
                break;
        }
    }
}