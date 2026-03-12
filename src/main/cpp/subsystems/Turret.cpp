#include "subsystems/Turret.h"

Turret::Turret() {
    //pids
    turretConfigs.Slot0.kP = ControllerConstants::turretkP; 
    turretConfigs.Slot0.kI = ControllerConstants::turretkI;
    turretConfigs.Slot0.kD = ControllerConstants::turretkD;

    turretConfigs.CurrentLimits.SupplyCurrentLimit = 10_A;
    turretConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    //turretConfigs.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    turretConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    turretMotor.SetPosition(0_tr);

    turretMotor.GetConfigurator().Apply(turretConfigs);
    encoder.GetConfigurator().Apply(encoderConfigs);

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

//for testing purposes without questnav
void Turret::addToSetpoint(double addition){
    setpoint += addition;
}

//stop motors
void Turret::stop(){
    turretMotor.StopMotor();
}

//angle of the turret found from the relative position of the motor
double Turret::getCurrentMotorAngle() {
    double motorPos = turretMotor.GetPosition().GetValueAsDouble() * 360; //find pos in degrees
    double currentAngle = std::fmod(motorPos/ControllerConstants::turretPulleyRatio, 360.0);//pullyRatio = 44
    return currentAngle;
}

//Not yet tested; We can get the relative value from the encoder, which makes life easier
double Turret::getCurrentEncoderAngle() {
    double encoderPos = encoder.GetPosition().GetValueAsDouble() * 360; //find pos in degrees
    double currentAngle = std::fmod(encoderPos/ControllerConstants::turretTbRatio, 360.0);//tb to turret ratio = 8.7777
    return currentAngle;
}

double Turret::getSetpoint(){
    return setpoint;
}

void Turret::setAngle(double targetAngle) {
    
    if (targetAngle > 180) {
        targetAngle = std::fmod(targetAngle, 360) - 360;
    } 

    else if (targetAngle < -180) {
        targetAngle = std::fmod(targetAngle, -360) + 360;
    } 
    turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t(targetAngle/360 * ControllerConstants::turretPulleyRatio)).WithSlot(0));

//Not yet tested! I need to test the encoder values first, and this is just a backup when skipping happens

//Should I use a while loop to make it stop when it hits the setpoint on the encoder value?

/*
    if (!isAtAngle(targetAngle)){
        if (targetAngle >= 0){
            setSpeed(0.1);
        }
        else if(targetAngle < 0){
            setSpeed(-0.1);
        }
    }
*/

}


bool Turret::isAtAngle(double targetAngle) {
    if (targetAngle > 180) {
        targetAngle = std::fmod(targetAngle, 360) - 360;
    } 

    else if (targetAngle < -180) {
        targetAngle = std::fmod(targetAngle, -360) + 360;
    } 
    double actualEncVal = getCurrentMotorAngle();
    if (std::fabs(targetAngle - actualEncVal) < 2) return true; // if the current angle is within 5 degrees of the target angle
    else return false;
}

void Turret::resetTurretPosition(){
    setAngle(0);
}

void Turret::ZeroTurret() {
    turretMotor.SetPosition(0_tr);
}