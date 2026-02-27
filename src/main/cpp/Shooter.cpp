#include "Shooter.h"

void Shooter::stop() {
    // Stop all motors
    ShooterMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    RackMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
}

void Shooter::zeroHood() {

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

    RackConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    RackConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    RackConfig.CurrentLimits.SupplyCurrentLimit = 5_A;
    RackConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

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
    // alter (0.775) based on how much of rotational velocity is translated to linear velocity
    float flywheelCircumference = ShooterConstants::PI * ShooterConstants::SHOOTERWHEELDIAMETER;
    float shooterVelocity = (shooterRPM * flywheelCircumference) / 60.0f;
    float exitVelo = 0.75f * shooterVelocity;

    // Target point (dx, dy) in meters
    // Alter (10) to make it shoot farther or closer to the center of the goal
    float dx = (10 * 0.0254f) + std::sqrt(std::pow(horizontalOffset * 0.0254f, 2.0f) + std::pow(yOffset * 0.0254f, 2.0f));
    const float dy = 72.0f * 0.0254f;
    const float verticalOffset = dy - (shooterHeight * 0.0254f);

    // Desired vertex location  
    // Alter (24) to make it shooter higher
    // const float VertexYPose = verticalOffset + (24.0f * 0.0254f);
    // float VertexXPose = dx - (24.0755062252f * 0.0254f);

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

int Shooter::findOptimalRPM(float horizontalOffset, float yOffset) {

    // 2d plane horizontal
    float dx = (10 * 0.0254f) + std::sqrt(std::pow(horizontalOffset * 0.0254f, 2.0f) + std::pow(yOffset * 0.0254f, 2.0f));

    if (dx < (25.0 * 0.0254f)) {
        return 0;
    } else if (dx < (50.0 * 0.0254f) && dx >= (25.0 * 0.0254f)) {
        return 1600;
    } else if (dx < (75.0 * 0.0254f) && dx >= (50.0 * 0.0254f)) {
        return 1650;
    } else if (dx < (100.0 * 0.0254f) && dx >= (75.0 * 0.0254f)) {
        return 1750;
    } else if (dx < (125.0 * 0.0254f) && dx >= (100.0 * 0.0254f)) {
        return 1900;
    } else if (dx < (150.0 * 0.0254f) && dx >= (125.0 * 0.0254f)) {
        return 2000;
    } else if (dx < (175.0 * 0.0254f) && dx >= (150.0 * 0.0254f)) {
        return 2100;
    } else {
        return 2250;
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
