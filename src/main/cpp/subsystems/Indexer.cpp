#include "subsystems/Indexer.h"

void Indexer::init(){
    indexerConfigs.Slot0.kP = 0.05;
    indexerConfigs.Slot0.kI = 0.0;
    indexerConfigs.Slot0.kD = 0.0;

    indexerConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    indexerConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;


    indexerConfigs.CurrentLimits.StatorCurrentLimit = 70_A;
    indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfigs.CurrentLimits.SupplyCurrentLimit = 40_A;
    indexerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    indexerMotor.GetConfigurator().Apply(indexerConfigs);
}

void Indexer::setIndexerSpeed(double speed){
    indexerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::angular_velocity::turns_per_second_t{speed}});
}

void Indexer::stopIndexer(){
    indexerMotor.Set(0);
}
