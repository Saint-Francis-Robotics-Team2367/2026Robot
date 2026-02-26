#include "subsystems/Turret.h"

Turret::Turret() {
    //pids
    turretConfigs.Slot0.kP = kP; 
    turretConfigs.Slot0.kI = kI;
    turretConfigs.Slot0.kD = kD;

    turretConfigs.CurrentLimits.SupplyCurrentLimit = 10_A;
    turretConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    //turretConfigs.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    turretConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    turretMotor.SetPosition(0_tr);

    turretMotor.GetConfigurator().Apply(turretConfigs);
    encoder.GetConfigurator().Apply(encoderConfigs);
    encoderOffset = encoderConfigs.MagnetSensor.MagnetOffset.value();

}

void Turret::addToSetpoint(double addition){
    setpoint += addition;
}

//stop motors
void Turret::stop(){
    turretMotor.StopMotor();
}
      
//set speed
void Turret::setSpeed(double spd){
    turretMotor.Set(spd);
    speed = spd;
}
void Turret::changeSpeed(double increment){
    speed += increment;
}

double Turret::getSpeed(){
    return speed;
}


//angle of the turret found by calculating corrected encoder position and dividing by pulley ratio and mod by 360
double Turret::getCurrentAngle() {
    double encoderPos = turretMotor.GetPosition().GetValueAsDouble() * 360; //find pos in degrees
    //double corrected = encoderPos - encoderOffset; //corrected value
    double currentAngle = std::fmod(encoderPos/pulleyRatio, 360.0);//divide by pulley ratio to convert from angle of small wheel to angle of large wheel
    return currentAngle;
}

/*
double Turret::findTurretAngle(){

}
*/
void Turret::updateTurret(double targetAngle){
    if (!isAtAngle(targetAngle)){
        setAngle(targetAngle);
    }
}

void Turret::setAngle(double targetAngle) {
    //double currentAngle = getCurrentAngle(); //get the current absolute angle
    /*
    if (fabs(currentAngle + targetAngle)>180){
        resetTurretPosition(); //this will take care of if the turret rotates more that 180 deg.
    }*/
/*
    double turnAngle = targetAngle - currentAngle; //find the angle the turret needs to turn
    //decides which direction to turn
    if (fabs(turnAngle) < 180){
        frc::SmartDashboard::PutNumber("turn angle", turnAngle);
        //turn to the desired position
        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t(turnAngle/360 * pulleyRatio)).WithSlot(0));
        smallPulleyCounter += turnAngle/360 * pulleyRatio; //keep track of how much the small pulley turns
    }
    else{
        //turn the opposite direction, change the turn angle
        frc::SmartDashboard::PutNumber("turn angle", turnAngle);
        turnAngle = (360-turnAngle) + currentAngle;
        //turn to the desired position
        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t (-turnAngle/360 * pulleyRatio)).WithSlot(0));
        smallPulleyCounter -= turnAngle/360 * pulleyRatio; //keep track of how much the small pulely turns
    }
    */
   //turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t(targetAngle/360 * pulleyRatio)).WithSlot(0));

//AFTER BASIC TESTS, TEST THIS LOGIC!!!

    //double turretAngle = getCurrentAngle()*360/pulleyRatio;
    
    if (targetAngle > 180) {
        targetAngle = targetAngle - 360;
    } 
    turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t(targetAngle/360 * pulleyRatio)).WithSlot(0));
/*
    double turretAngle = getCurrentAngle();
    if (std::fabs(targetAngle + turretAngle) > 180){
        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t(-(360-targetAngle)/360 * pulleyRatio)).WithSlot(0));
        smallPulleyCounter += -(360-targetAngle)/360 * pulleyRatio;
    }
    else if (std::fabs(targetAngle + turretAngle) < 180){
        turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t(targetAngle/360 * pulleyRatio)).WithSlot(0));
        smallPulleyCounter += targetAngle * pulleyRatio/360;
    }
    */



/*
    double turretAngle = getCurre
*/
}


bool Turret::isAtAngle(double targetAngle) {
    double encoderVal = fmod((pulleyRatio * targetAngle), 360);
    double actualEncVal = getCurrentAngle();
    if (std::fabs(encoderVal - actualEncVal) < 2) return true; // if the current angle is within 5 degrees of the target angle
    else return false;
}

//NOT GOOD
void Turret::resetTurretPosition(){
    double target = 0;
    double currentAngle = getCurrentAngle();
    //use the small pully counter to see how many turns the encoder did, convert that to degrees the turret turns
    if(std::fabs(smallPulleyCounter*360/pulleyRatio)>180){
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
/*
void Turret::correctionFunction(double target){
    double currTBPos = getCurrentAngle(); //this should be the throughbore position when it it fixed
    double targetTBPos = std::fmod(target*tbTurretrRatio, 360);
    if (currTBPos )
}
*/

//double Turret::getShortestAngle()


