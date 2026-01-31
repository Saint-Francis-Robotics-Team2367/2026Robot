#include "Turret.h"


void Turret::init() {
    turretConfigs.Slot0.kP = kP; 
    turretConfigs.Slot0.kI = kI;
    turretConfigs.Slot0.kD = kD;
   
    turretMotor.GetConfigurator().Apply(turretConfigs);
    encoderConfigs.MagnetSensor.WithSensorDirection(ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);
    encoderOffset = encoderConfigs.MagnetSensor.MagnetOffset.value();
}


//stop motors
void Turret::stop(){
    turretMotor.StopMotor();
    turretMotor.GetPosition();
}
      
      
void Turret::setSpeed(double speed){
    turretMotor.Set(speed);
}

//angle of the turret
double Turret::getCurrentAngle() {
    double encoderPos = turretMotor.GetPosition().GetValueAsDouble() * 360;
    double corrected = encoderPos - encoderOffset; //do i need this?
    double currentAngle = fmod(corrected/gearRatio, 360.0);
    return currentAngle;
}


void Turret::setAngle(double targetAngle) {

    double currentAngle = getCurrentAngle();
    double turnAngle = targetAngle - currentAngle;

    if (fabs(turnAngle) <= 180){

        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t(turnAngle/360 * gearRatio)).WithSlot(0));
        //double output = pid.Calculate(currentAngle, targetAngle);
    }
    else{

        turnAngle = (360-turnAngle) + currentAngle;
        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t (-turnAngle/360 * gearRatio)).WithSlot(0));
       //double output = -pid.Calculate(currentAngle, 2*PI - targetAngle);
    }
}

bool Turret::isAtAngle(double targetAngle) {
    if (fabs(targetAngle - getCurrentAngle()) < 5) return true; // if the current angle is within 5 degrees of the target angle
    else return false;
}

void Turret::resetTurretPosition(){
    double currentAngle = getCurrentAngle();
    if (currentAngle < 180){
        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t (-currentAngle/360 * gearRatio)).WithSlot(0)); //rotate back the opposite way
    }
    else {
        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t ((360-currentAngle)/360 * gearRatio)).WithSlot(0)); //rotate back the opposite way
    }
}
    
