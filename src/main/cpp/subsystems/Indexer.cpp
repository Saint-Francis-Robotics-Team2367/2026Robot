#include "subsystems/Indexer.h"

// Initalization of Motor and Configs
void Indexer::init(){
    indexerConfigs.Slot0.kP = 0.05;
    indexerConfigs.Slot0.kI = 0.0;
    indexerConfigs.Slot0.kD = 0.0;
    indexerConfigs.Slot0.kV = 0.0;

    indexerConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    indexerConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    indexerConfigs.CurrentLimits.StatorCurrentLimit = 20_A;
    indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfigs.CurrentLimits.SupplyCurrentLimit = 20_A;
    indexerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    indexerMotor.GetConfigurator().Apply(indexerConfigs);
}

frc2::CommandPtr Indexer::index(){
    return frc2::cmd::Run([this] {setIndexerSpeed(250);}, {this});
}

// Gear Ratio is 1:1, so RPM is directly proportional to the output speed of the indexer
void Indexer::setIndexerSpeed(double indexerRPM){
    indexerMotor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{units::angular_velocity::turns_per_second_t{indexerRPM / 60.0}});
}


void Indexer::stopIndexer(){
    indexerMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
}