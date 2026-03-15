#include "subsystems/Turret.h"
#include "subsystems/vision/QuestNav.h"

void Turret::init() {
    //pids
    turretConfigs.Slot0.kP = TurretConstants::turretkP; 
    turretConfigs.Slot0.kI = TurretConstants::turretkI;
    turretConfigs.Slot0.kD = TurretConstants::turretkD;

    turretConfigs.CurrentLimits.SupplyCurrentLimit = 15_A;
    turretConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretConfigs.CurrentLimits.StatorCurrentLimit = 40_A;
    turretConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    //turretConfigs.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    turretConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
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
    turretMotor.Set(speed);
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
    double currentAngle = std::fmod(motorPos/TurretConstants::turretPulleyRatio, 360.0);//pullyRatio = 44
    if (currentAngle > 180.0)  currentAngle -= 360.0;
    if (currentAngle < -180.0) currentAngle += 360.0;
    return currentAngle;
}

//Not yet tested; We can get the relative value from the encoder, which makes life easier
double Turret::getCurrentEncoderAngle() {
    double encoderPos = encoder.GetPosition().GetValueAsDouble() * 360; //find pos in degrees
    double currentAngle = std::fmod(encoderPos/TurretConstants::turretTbRatio, 360.0);//tb to turret ratio = 8.7777
    if (currentAngle > 180.0)  currentAngle -= 360.0;
    if (currentAngle < -180.0) currentAngle += 360.0;
    return currentAngle;
}

double Turret::getSetpoint(){
    return setpoint;
}

void Turret::autoMoveToTarget() {
    // Convert robot pose from meters to inches to match hub coordinate constants
    double robotX_in = mDrive.getPose().X().value() * ShooterConstants::MeterToInches;
    double robotY_in = mDrive.getPose().Y().value() * ShooterConstants::MeterToInches;
    double dx = TurretConstants::hubX - robotX_in;
    double dy = TurretConstants::hubY - robotY_in;

    // atan2(dx, dy) gives angle from +Y (forward) axis, CW-positive toward +X (right).
    // Negate to make CCW-positive so it matches the robotHeading convention (0 = facing forward/+Y, CCW+).
    double angleToHub = -atan2(dx, dy) * 180.0 / M_PI;
    double robotHeading = QuestNav::getInstance().getPose2d().Rotation().Degrees().value();
    double turretTarget = angleToHub - robotHeading;

    while (turretTarget > 180)  turretTarget -= 360;
    while (turretTarget < -180) turretTarget += 360;

    double clampedTarget = std::clamp(turretTarget, -TurretConstants::turretMaxAngle, TurretConstants::turretMaxAngle);

    frc::SmartDashboard::PutNumber("turret angle", turretTarget);
    frc::SmartDashboard::PutBoolean("is angle in range?", turretTarget == clampedTarget);
    
    turretMotor.SetControl(
        positionVoltage
            .WithPosition(units::angle::turn_t(clampedTarget / 360 * TurretConstants::turretPulleyRatio))
            .WithSlot(0));
}


void Turret::setAngle(double targetAngle) {
    double clampedTarget = std::clamp(targetAngle, -TurretConstants::turretMaxAngle, TurretConstants::turretMaxAngle);
    turretMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t(clampedTarget / 360 * TurretConstants::turretPulleyRatio)).WithSlot(0));
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

/*
void Turret::autoTarget() {
    double heading = QuestNav::getInstance().getPose2d().Rotation().Degrees().value();
    double targetDeg = (heading - TurretConstants::hubHeading);
    setAngle(targetDeg);
}
*/