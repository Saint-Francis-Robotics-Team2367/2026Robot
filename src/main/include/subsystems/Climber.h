#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/configs/Configuration.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"

#include "frc/Servo.h"

#include "Constants.h"

class Climber {
private:
    ctre::phoenix6::controls::PositionVoltage positionVoltage{0_tr};
    ctre::phoenix6::controls::VelocityVoltage velocityVoltage{0_tps};

    ctre::phoenix6::hardware::TalonFX leftKrakenMotor{ClimberConstants::leftKrakenID};
    ctre::phoenix6::hardware::TalonFX rightKrakenMotor{ClimberConstants::rightKrakenID};

    ctre::phoenix6::configs::TalonFXConfiguration leftKrakenConfigs{};
    ctre::phoenix6::configs::TalonFXConfiguration rightKrakenConfigs{};

    frc::Servo leftServo{ClimberConstants::leftServoID};
    frc::Servo rightServo{ClimberConstants::rightServoID};

public:
    void init();
    void setPosition(float position);
    void setVelocity(float velocity);
    void setServoPosition(float position);
};