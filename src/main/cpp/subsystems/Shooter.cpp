#include "subsystems/Shooter.h"

void Shooter::stop() {
    // Stop all motors
    ShooterMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    RackMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
}

void Shooter::moveHoodToZero() {

    // Define "center" as wherever the rack motor is RIGHT NOW
    // test this
    // RackMotor.SetControl(positionVoltage.WithPosition(units::angle::turn_t{-1*targetAbs}).WithSlot(0));
    RackMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{units::angle::turn_t{0.0}});
}

void Shooter::init() {
    // Configure PID constants for Flywheel Master (Velocity Control)
    FlywheelConfig.Slot0.kP = ShooterConstants::FlywheelP;
    FlywheelConfig.Slot0.kI = ShooterConstants::FlywheelI;
    FlywheelConfig.Slot0.kD = ShooterConstants::FlywheelD;
    FlywheelConfig.Slot0.kV = ShooterConstants::FlywheelV;

    FlywheelConfig.CurrentLimits.SupplyCurrentLimit = 20_A;
    FlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    FlywheelConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;


    // Configure PID constants for Rack Motor (Position Control)
    RackConfig.Slot0.kP = ShooterConstants::RackP;
    RackConfig.Slot0.kI = ShooterConstants::RackI;
    RackConfig.Slot0.kD = ShooterConstants::RackD;
    RackConfig.Slot0.kG = ShooterConstants::RackG;

    RackConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    RackConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    RackConfig.CurrentLimits.SupplyCurrentLimit = 5_A;
    RackConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    RackConfig.CurrentLimits.StatorCurrentLimit = 10_A;
    RackConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Apply configurations
    ShooterMotor.GetConfigurator().Apply(FlywheelConfig);
    RackMotor.GetConfigurator().Apply(RackConfig);

    RackMotor.SetPosition(0_tr);
    hoodCenterRot = RackMotor.GetPosition().GetValueAsDouble();
}

bool Shooter::setFlywheelSpeed(float shooterRPM) {
    // set shooter velocity
    ShooterMotor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{units::angular_velocity::turns_per_second_t{shooterRPM / 60.0}});

    float targetVelocity = shooterRPM / 60.0;
    float actualVelocity = ShooterMotor.GetVelocity().GetValue().value();
    const float tolerance = 0.05; // or whatever tolerance you want

    if (std::fabs(targetVelocity - actualVelocity) < tolerance) {
        std::cout << "Shooter at target speed." << std::endl;
        return true;
    } else {
        std::cout << "Shooter NOT at target speed." << std::endl;
        return false;
    }
}



// initial angle is the angle of the hood at 0 degrees of rack rotation
// all units are in inches
void Shooter::setHoodPosition(float shooterRPM, float horizontalOffset, float yOffset, float shooterHeight, float initialAngle, float minAngle, float MotorGearRatio, float ThroughBoreGearRatio) {

    // Convert shooter RPM to linear velocity (m/s)
    // alter (0.75) based on how much of rotational velocity is translated to linear velocity
    float flywheelCircumference = ShooterConstants::PI * ShooterConstants::SHOOTERWHEELDIAMETER;
    float shooterVelocity = (shooterRPM * flywheelCircumference) / 60.0f;
    float exitVelo = shooterVelocity;

    // Target point (dx, dy) in meters
    // Alter (10) to make it shoot farther or closer to the center of the goal
    float dx = (7.5 * ShooterConstants::MeterConversionFactor) + std::sqrt(std::pow(horizontalOffset * ShooterConstants::MeterConversionFactor, 2.0f) + std::pow(yOffset * ShooterConstants::MeterConversionFactor, 2.0f));
    const float dy = 72.0f * ShooterConstants::MeterConversionFactor;
    const float verticalOffset = dy - (shooterHeight * ShooterConstants::MeterConversionFactor);

    // Desired vertex location  
    // Alter (24) to make it shooter higher
    // const float VertexYPose = verticalOffset + (24.0f * ShooterConstants::MeterConversionFactor);
    // float VertexXPose = dx - (24.0755062252f * ShooterConstants::MeterConversionFactor);

    // Solve projectile equation at (dx, dy)
    const float A = (ShooterConstants::GRAVITY * dx * dx) / (2.0f * exitVelo * exitVelo);
    const float discriminant = dx * dx - 4.0f * A * (A + verticalOffset);

    if (discriminant < 0.0f || A == 0.0f) {
        std::cout << "No valid hood angle found (discriminant < 0)." << std::endl;
        return;
    }

    const float sqrtDisc = std::sqrt(discriminant);

    const float t1 = (dx - sqrtDisc) / (2.0f * A);
    const float t2 = (dx + sqrtDisc) / (2.0f * A);

    const float theta1 = std::atan(t1);
    const float theta2 = std::atan(t2);

    auto apexHeight = [&](float theta) {
    float s = std::sin(theta);
    return (exitVelo * exitVelo * s * s) / (2.0f * ShooterConstants::GRAVITY);
    };

    // theta1/theta2 are radians (projectile launch angles)
    float hood1Deg = theta1 * 180.0f / ShooterConstants::PI;
    float hood2Deg = theta2 * 180.0f / ShooterConstants::PI;

    // Validate in HOOD degrees (since minAngle/initialAngle are degrees)
    bool v1 = hood1Deg >= minAngle && hood1Deg <= initialAngle;
    bool v2 = hood2Deg >= minAngle && hood2Deg <= initialAngle;

    if (!(v1 || v2)) {
        std::cout << "No valid hood angle found (out of hood limits)." << std::endl;
        return;
    }

    float unknownAngle;

    if (v1 && v2) {
        float h1 = apexHeight(theta1);
        float h2 = apexHeight(theta2);
        unknownAngle = (h1 >= h2) ? theta1 : theta2; 
    } else if (v1) {
        unknownAngle = theta1;
    } else {
        unknownAngle = theta2;
    }

    // Convert chosen projectile angle (radians) -> hood angle (degrees)
    float hoodAngleDegrees = unknownAngle * 180.0f / ShooterConstants::PI;

    // Convert hood angle to motor turns (absolute command)
    // Assumes MotorGearRatio = motor turns per 1 hood revolution
    float deltaDeg = hoodAngleDegrees - initialAngle;
    double motorTurnsTarget = (deltaDeg / 360.0) * MotorGearRatio;
    double targetAbsLocal = hoodCenterRot + motorTurnsTarget;

    targetAbs = targetAbsLocal;

    RackMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{units::angle::turn_t{targetAbs}});

}

float Shooter::findOptimalRPM(float horizontalOffset, float yOffset) {
    float dx = (10.0f * ShooterConstants::MeterConversionFactor) + std::sqrt(std::pow(horizontalOffset * ShooterConstants::MeterConversionFactor, 2.0f) + std::pow(yOffset* ShooterConstants::MeterConversionFactor, 2.0f));

    if (dx < (25.0f * ShooterConstants::MeterConversionFactor)) {
        return 0.0f;
    } else if (dx < (50.0f * ShooterConstants::MeterConversionFactor)) {
        return 911.3741f / ShooterConstants::SHOOTEREFFICIENCY;
        
    } else if (dx < (75.0f * ShooterConstants::MeterConversionFactor)) {
        return 1005.0751f / ShooterConstants::SHOOTEREFFICIENCY;

    } else if (dx < (100.0f * ShooterConstants::MeterConversionFactor)) {
        return 1101.0902f / ShooterConstants::SHOOTEREFFICIENCY;

    } else if (dx < (125.0f * ShooterConstants::MeterConversionFactor)) {
        return 1193.5726f / ShooterConstants::SHOOTEREFFICIENCY;

    } else if (dx < (150.0f * ShooterConstants::MeterConversionFactor)) {
        return 1281.3769f / ShooterConstants::SHOOTEREFFICIENCY;

    } else if (dx < (175.0f * ShooterConstants::MeterConversionFactor)) {
        return 1364.6042f / ShooterConstants::SHOOTEREFFICIENCY;

    } else {
        return 1443.6638f / ShooterConstants::SHOOTEREFFICIENCY; 
    }
}


void Shooter::applyHoodBrake() {
    static ctre::phoenix6::controls::TorqueCurrentFOC holdTorque{0_A};
    constexpr units::current::ampere_t kHoldCurrent = 15_A;
    RackMotor.SetControl(holdTorque.WithOutput(kHoldCurrent));
}

void Shooter::releaseHoodBrake() {
    RackMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0});
}

double Shooter::getShooterVelocity() { // rpm
    return std::abs(ShooterMotor.GetVelocity().GetValueAsDouble()) * 60.0;
}

void Shooter::ZeroHood() {
    RackMotor.SetPosition(0_tr);
}

void Shooter::setManualHoodPosition(float targetAngle) {
    double initialAngle = findHoodAngle();
    // Convert chosen projectile angle (radians) -> hood angle (degrees)
    float hoodAngleDegrees = targetAngle * 180.0f / ShooterConstants::PI;

    // Convert hood angle to motor turns (absolute command)
    // Assumes MotorGearRatio = motor turns per 1 hood revolution
    float deltaDeg = hoodAngleDegrees - initialAngle; 
    double motorTurnsTarget = (deltaDeg / 360.0) * ShooterConstants::motorGearRatio;
    double targetAbsLocal = hoodCenterRot + motorTurnsTarget;

    targetAbs = targetAbsLocal;

    RackMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{units::angle::turn_t{targetAbs}});
}

double Shooter::findHoodAngle() {
    return RackMotor.GetPosition().GetValueAsDouble() / ShooterConstants::motorGearRatio;
}