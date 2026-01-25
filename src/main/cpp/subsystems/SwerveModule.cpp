#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int driveMotorID, int steerMotorID, int encoderID, std::string canBus) : driveMotor(driveMotorID, canBus),
                                                                                                            steerMotor(steerMotorID, canBus),
                                                                                                            encoder(encoderID, canBus) {};

void SwerveModule::setMotor(control controlType, MotorType motorType, double input) {
    switch (controlType) {
        case control::VELOCITY:
            if (motorType == MotorType::DRIVE) {
                driveMotor.SetControl(velocityVoltage.WithVelocity(units::turns_per_second_t(input / 60.0)));
            }
            else {
                steerMotor.SetControl(velocityVoltage.WithVelocity(units::turns_per_second_t(input / 60.0)));
            }
            break;
        
        case control::POSITION: 
            if (motorType == MotorType::DRIVE) {
                driveMotor.SetControl(positionVoltage.WithPosition(units::turn_t(input)));
            }
            else {
                steerMotor.SetControl(positionVoltage.WithPosition(units::turn_t(input)));
            }
    }
}

void SwerveModule::stopModule() {
    setMotor(VELOCITY, DRIVE, 0.0);
    setMotor(VELOCITY, STEER, 0.0);
}

void SwerveModule::initHardware() {
    // Drive Configs
    driveConfigs.Slot0.kP = 0.11;   // An error of 1 rotation per second results in 2V output
    driveConfigs.Slot0.kI = 0.0;    // An error of 1 rotation per second increases output by 0.5V every second
    driveConfigs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.0001 volts output
    driveConfigs.Slot0.kV = 0.0;   // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    driveConfigs.Slot0.kS = 0.00;

    driveConfigs.TorqueCurrent.PeakForwardTorqueCurrent = units::ampere_t(10);
    driveConfigs.TorqueCurrent.PeakReverseTorqueCurrent = units::ampere_t(-10);

    driveConfigs.CurrentLimits.SupplyCurrentLimit = 40.0_A;
    driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveConfigs.CurrentLimits.StatorCurrentLimit = 80.0_A;
    driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    driveConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    driveConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    driveConfigs.MotorOutput.PeakForwardDutyCycle = 1.0;  
    driveConfigs.MotorOutput.PeakReverseDutyCycle = -1.0;
    driveMotor.GetConfigurator().Apply(driveConfigs);


    // Steer Configs
    steerConfigs.Slot0.kP = 0.3;   // An error of 1 rotation per second results in 2V output
    steerConfigs.Slot0.kI = 0.0;    // An error of 1 rotation per second increases output by 0.5V every second
    steerConfigs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.0001 volts output
    steerConfigs.Slot0.kV = 0.0;   // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    steerConfigs.Slot0.kS = 0.00;

    steerConfigs.TorqueCurrent.PeakForwardTorqueCurrent = units::ampere_t(10);
    steerConfigs.TorqueCurrent.PeakReverseTorqueCurrent = units::ampere_t(-10);

    steerConfigs.CurrentLimits.SupplyCurrentLimit = 20.0_A;
    steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    steerConfigs.CurrentLimits.StatorCurrentLimit = 55.0_A;
    steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    steerConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    steerConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    steerConfigs.MotorOutput.PeakForwardDutyCycle = 1.0;  
    steerConfigs.MotorOutput.PeakReverseDutyCycle = -1.0;
    steerMotor.GetConfigurator().Apply(steerConfigs);


    // Encoder
    encoderConfigs.MagnetSensor.WithSensorDirection(ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);
    moduleOffset = encoderConfigs.MagnetSensor.MagnetOffset.value();
}

void SwerveModule::invertModule(ctre::phoenix6::signals::InvertedValue value, bool steer, bool drive) {
    if (steer) {
        steerConfigs.MotorOutput.Inverted = value;
        steerMotor.GetConfigurator().Apply(steerConfigs);
    }
    if (drive) {
        driveConfigs.MotorOutput.Inverted = value;
        driveMotor.GetConfigurator().Apply(driveConfigs);
    }
}

void SwerveModule::setDesiredState(frc::SwerveModuleState& state) {
    auto optimizedState = frc::SwerveModuleState::Optimize(state, units::radian_t(encoder.GetAbsolutePosition().GetValueAsDouble() * MathConstants::TWO_PI));

    setMotor(VELOCITY, DRIVE, optimizedState.speed.value());
    setMotor(POSITION, STEER, (optimizedState.angle.Radians().value() / MathConstants::TWO_PI));
}

frc::SwerveModulePosition SwerveModule::getPosition() {
    double conversionFactor = M_PI * 0.1016 * ModuleConstants::kDriveGearRatio;
    return frc::SwerveModulePosition{units::meter_t(driveMotor.GetPosition().GetValueAsDouble() * conversionFactor),
                                     units::radian_t(encoder.GetAbsolutePosition().GetValueAsDouble() * MathConstants::TWO_PI - moduleOffset)
    };
}