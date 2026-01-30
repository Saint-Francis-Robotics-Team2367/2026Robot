#include "Turret.h"


void Turret::init() {
    cfg.Slot0.kP = kP; 
    cfg.Slot0.kI = kI;
    cfg.Slot0.kD = kD;
   
    turretMotor.GetConfigurator().Apply(cfg);
    encoderConfigs.MagnetSensor.WithSensorDirection(ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);
}


//stop motors
void Turret::stop(){
    TurretMotor.Disable();
}
      
      
void Turret::setSpeed(double speed){
    TurretMotor.set(speed);
}
      
      
//angle of the turret
double Turret::getCurrentAngle() {
    double encoderPos = encoder.getAbsolutePosition().getDegrees();
    double corrected = encoderPos - encoderOffset;
    double currentAngle = fmod(corrected/gearRatio, 360.0);
    return currentAngle;
}


void Turret::setAngle(double targetAngle) {

    double currentAngle = getCurrentAngle() - ;
    double turnAngle = targetAngle - currentAngle;

    if (fabs(turnAngle) <= 180){

        motor.SetControl(positionVoltage.WithPosition(units::angle::turn_t(turnAngle)).WithSlot(0));
        //double output = pid.Calculate(currentAngle, targetAngle);
    }
    else{

        turnAngle =  (360-turnAngle) + currentAngle;
        motor.SetControl(positionVoltage.WithPosition(units::angle::turn_t (-turnAngle)).WithSlot(0));
       //double output = -pid.Calculate(currentAngle, 2*PI - targetAngle);
    }
}


bool Turret::isAtAngle(double targetAngle) {
    if (getCurrentAngle() == targetAngle) return true;
    else return false;
}

void Turret::resetTurretPosition(){
    double currentAngle = getCurrentAngle();
    corrected = currentAngle - encoderOffset;
    motor.SetControl(positionVoltage.WithPosition(units::angle::turn_t (-corrected)).WithSlot(0));
}




//1. needs to reset everytime angle change exceeds 180
//2. need to use the encoder to correct the pid

