#include "Turret.h"

void Turret::init() {
    //pids
    turretConfigs.Slot0.kP = kP; 
    turretConfigs.Slot0.kI = kI;
    turretConfigs.Slot0.kD = kD;
   
    turretMotor.GetConfigurator().Apply(turretConfigs);

    //through bore
    encoderConfigs.MagnetSensor.WithSensorDirection(ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);
    encoderOffset = encoderConfigs.MagnetSensor.MagnetOffset.value();
}


//stop motors
void Turret::stop(){
    turretMotor.StopMotor();
}
      
//set speed
void Turret::setSpeed(double speed){
    turretMotor.Set(speed);
}

//angle of the turret found by calculating corrected encoder position and dividing by pulley ratio and mod by 360
double Turret::getCurrentAngle() {
    double encoderPos = turretMotor.GetPosition().GetValueAsDouble() * 360; //find pos in degrees
    double corrected = encoderPos - encoderOffset; //corrected value
    double currentAngle = fmod(corrected/pulleyRatio, 360.0);//divide by pulley ratio to convert from angle of small wheel to angle of large wheel

    return currentAngle;
}


void Turret::setAngle(double targetAngle) {
    resetTurretPosition(); //this will take care of if the turret rotates more that 180 deg.

    double currentAngle = getCurrentAngle(); //get the current absolute angle
    double turnAngle = targetAngle - currentAngle; //find the angle the turret needs to turn
    //decides which direction to turn
    if (fabs(turnAngle) <= 180){
        //turn to the desired position
        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t(turnAngle/360 * pulleyRatio)).WithSlot(0));
        smallPulleyCounter += turnAngle/360 * pulleyRatio; //keep track of how much the small pulley turns
    }
    else{
        //turn the opposite direction, change the turn angle
        turnAngle = (360-turnAngle) + currentAngle;
        //turn to the desired position
        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t (-turnAngle/360 * pulleyRatio)).WithSlot(0));
        smallPulleyCounter -= turnAngle/360 * pulleyRatio; //keep track of how much the small pulely turns
    }
}

bool Turret::isAtAngle(double targetAngle) {
    if (fabs(targetAngle - getCurrentAngle()) < 5) return true; // if the current angle is within 5 degrees of the target angle
    else return false;
}

void Turret::resetTurretPosition(){
    double target = 0;
    double currentAngle = getCurrentAngle();
    //use the small pully counter to see how many turns the encoder did, convert that to degrees the turret turns
    if(fabs(smallPulleyCounter*360/pulleyRatio)>180){
        target = -currentAngle/360 * pulleyRatio; //set target angle
        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t (target)).WithSlot(0)); //rotate back the opposite way
        smallPulleyCounter += target; //update small pulley counter
    }
    else {
        target = (360-currentAngle)/360 * pulleyRatio; //set target angle
        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t (target)).WithSlot(0)); //rotate back the opposite way
        smallPulleyCounter += target; //update small pulley counter
    }
}

