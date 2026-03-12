#include "subsystems/Indexer.h"

// Initalization of Motor and Configs
void Indexer::init(){
    indexerConfigs.Slot0.kP = 0.5;
    indexerConfigs.Slot0.kI = 0.0;
    indexerConfigs.Slot0.kD = 0.0;
    indexerConfigs.Slot0.kV = 0.0;

    indexerConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    indexerConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    indexerConfigs.CurrentLimits.StatorCurrentLimit = 65_A;
    indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfigs.CurrentLimits.SupplyCurrentLimit = 25_A;
    indexerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 10_A;
    indexerConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -10_A;

    indexerMotor.GetConfigurator().Apply(indexerConfigs);
}


// Gear Ratio is 1:1, so RPM is directly proportional to the output speed of the indexer
void Indexer::setIndexerSpeed(double indexerRPM){
    indexerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::angular_velocity::turns_per_second_t{indexerRPM / 60.0}}.WithSlot(0));
}


void Indexer::stopIndexer(){
    indexerMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
}

frc2::CommandPtr Indexer::RunIndexer(Indexer* indexer, double speed) {
    return frc2::cmd::StartEnd(
        [indexer, speed] {indexer->setIndexerSpeed(speed);},
        [indexer] {indexer->stopIndexer();},
        {indexer}
    );
}

bool Indexer::IndexerStall() {
    double indexerTorque = indexerMotor.GetTorqueCurrent().GetValueAsDouble();
    return (indexerTorque > IndexerConstants::indexerStallAmps);
}