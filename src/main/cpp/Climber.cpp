#include "Climber.h"

void Climber::init(){
    config.Inverted(false);
    config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    config.closedLoop.Pid(0.035, 0, 0.0);
    config.SmartCurrentLimit(55);

    config2.Inverted(true);
    config2.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    config2.closedLoop.Pid(0.035, 0, 0.0);
    config2.SmartCurrentLimit(55);

    motor1.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    motor2.Configure(config2, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    enc1.SetPosition(0);
    enc2.SetPosition(0);
}
void Climber::setState(double position){
    motorctr1.SetReference(position, rev::spark::SparkLowLevel::ControlType::kPosition);
    motorctr2.SetReference(position, rev::spark::SparkLowLevel::ControlType::kPosition);
}
void Climber::climb(){
    motor1.Set(1.0);
    motor2.Set(1.0);
}
void Climber::stop(){
    motor1.Set(0);
    motor2.Set(0);
}   