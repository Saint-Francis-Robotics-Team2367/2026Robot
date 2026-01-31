#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/configs/Configuration.hpp>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

class SwerveModule {
private:
    ctre::phoenix6::hardware::TalonFX driveMotor;
    ctre::phoenix6::hardware::TalonFX steerMotor;
    ctre::phoenix6::hardware::CANcoder encoder;

    ctre::phoenix6::controls::VelocityVoltage velocityVoltage{0_tps}; //turns per second
    ctre::phoenix6::controls::PositionVoltage positionVoltage{0_tr}; //turns

    //configurations for drive/steer motors and encoders
    ctre::phoenix6::configs::TalonFXConfiguration driveConfigs{};
    ctre::phoenix6::configs::TalonFXConfiguration steerConfigs{};
    ctre::phoenix6::configs::CANcoderConfiguration encoderConfigs{};

public:
    double moduleOffset = 0.0; // Rotations

    //control motors with VELOCITY or POSITION
    enum control {
        VELOCITY,
        POSITION
    };

    //motor can either be STEER or DRIVE
    enum MotorType {
        STEER,
        DRIVE
    };


    SwerveModule(int driveMotorID, int steerMotorID, int encoderID, std::string canBus = "rio"); //canBus can also be canivore

    void setMotor(control controlType, MotorType motorType, double input);
    void stopModule();
    void initHardware();
    void invertModule(ctre::phoenix6::signals::InvertedValue value, bool steer, bool drive); // Defaulted to CounterClockwise Positive
    void setDesiredState(frc::SwerveModuleState& state);
    frc::SwerveModulePosition getPosition();
    void zeroModule();
};