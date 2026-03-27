#include "subsystems/Spindexer.h"

// Initalization of Motor and Configs
void Spindexer::init(){
    spindexerConfigs.Slot0.kP = 0.5;
    spindexerConfigs.Slot0.kI = 0.0;
    spindexerConfigs.Slot0.kD = 0.0;
    spindexerConfigs.Slot0.kV = 0.0;

    spindexerConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    spindexerConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    spindexerConfigs.CurrentLimits.StatorCurrentLimit = 45_A;
    spindexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    spindexerConfigs.CurrentLimits.SupplyCurrentLimit = 15_A;
    spindexerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    spindexerConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 10_A;
    spindexerConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -10_A;

    spindexerMotor.GetConfigurator().Apply(spindexerConfigs);
}


// Gear Ratio is 1:1, so RPM is directly proportional to the output speed of the indexer
void Spindexer::setSpindexerSpeed(double indexerRPM){
    spindexerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::angular_velocity::turns_per_second_t{indexerRPM / 60.0}}.WithSlot(0));
}

void Spindexer::stopSpindexer(){
    spindexerMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
}

frc2::CommandPtr Spindexer::RunSpindexer(Spindexer* spindexer, double speed) {
    return frc2::cmd::StartEnd(
        [spindexer, speed] {spindexer->setSpindexerSpeed(speed);},
        [spindexer] {spindexer->stopSpindexer();},
        {spindexer}
    );
}

bool Spindexer::SpindexerStall() {
    double statorCurrent = spindexerMotor.GetStatorCurrent().GetValueAsDouble();
    double velocity = spindexerMotor.GetVelocity().GetValueAsDouble();
    return (statorCurrent > SpindexerConstants::spindexerStallCurrent) &&
           (std::abs(velocity) < SpindexerConstants::spindexerStallVelocityThreshold);
}

void Spindexer::DisplayValues() {
    frc::SmartDashboard::PutNumber("Stator Current", spindexerMotor.GetStatorCurrent().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Torque Current", spindexerMotor.GetTorqueCurrent().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Indexer Velocity", spindexerMotor.GetVelocity().GetValueAsDouble());
}