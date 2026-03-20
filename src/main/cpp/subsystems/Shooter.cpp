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
    FlywheelConfig.Slot0.kS = ShooterConstants::FlywheelS;

    FlywheelConfig.CurrentLimits.SupplyCurrentLimit = 35_A;
    FlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    FlywheelConfig.CurrentLimits.StatorCurrentLimit = 40_A;
    FlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    FlywheelConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;


    // Configure PID constants for Rack Motor (Position Control)
    RackConfig.Slot0.kP = ShooterConstants::RackP;
    RackConfig.Slot0.kI = ShooterConstants::RackI;
    RackConfig.Slot0.kD = ShooterConstants::RackD;
    RackConfig.Slot0.kG = ShooterConstants::RackG;

    RackConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    RackConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

    RackConfig.CurrentLimits.SupplyCurrentLimit = 10_A;
    RackConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    RackConfig.CurrentLimits.StatorCurrentLimit = 20_A;
    RackConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Apply configurations
    ShooterMotor.GetConfigurator().Apply(FlywheelConfig);
    RackMotor.GetConfigurator().Apply(RackConfig);

    RackMotor.SetPosition(0_tr);
    hoodCenterRot = RackMotor.GetPosition().GetValueAsDouble();
}

bool Shooter::setFlywheelSpeed(float shooterRPM) {
    // set shooter velocity
    float efficientRPM = shooterRPM / ShooterConstants::SHOOTEREFFICIENCY;
    ShooterMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::angular_velocity::turns_per_second_t{efficientRPM/ 60.0}});

    // float targetVelocity = efficientRPM / 60.0;
    // float actualVelocity = ShooterMotor.GetVelocity().GetValue().value();
    // const float tolerance = 3; // 3 TPS ≈ 180 RPM

    // if (std::fabs(targetVelocity - actualVelocity) < tolerance) {
    //     // std::cout << "Shooter at target speed." << std::endl;
    //     return true;
    // } else {
    //     // std::cout << "Shooter NOT at target speed." << std::endl;
    //     return false;
    // }
    return true;
}

// initial angle is the angle of the hood at 0 degrees of rack rotation
// all units are in meters
void Shooter::setHoodPosition(float shooterRPM, float horizontalOffset, float yOffset, float shooterHeight, float initialAngle, float minAngle, float MotorGearRatio, float ThroughBoreGearRatio) {

    // Convert shooter RPM to linear velocity (m/s)
    // alter (0.75) based on how much of rotational velocity is translated to linear velocity
    float flywheelCircumference = ShooterConstants::PI * ShooterConstants::SHOOTERWHEELDIAMETER;
    float shooterVelocity = (shooterRPM * flywheelCircumference) / 60.0f;
    float exitVelo = shooterVelocity;

    // Target point (dx, dy) in meters
    // Alter (num) to make it shoot farther or closer to the center of the goal
    float dx = 0.0254f + std::sqrt(horizontalOffset * horizontalOffset + yOffset * yOffset);
    frc::SmartDashboard::PutNumber("Shooter Distance (dx)", dx);
    const float dy = 1.8288f;  // 72.0 inches in meters
    const float verticalOffset = dy - shooterHeight;

    // Desired vertex location  
    // Alter (24) to make it shooter higher
    // const float VertexYPose = verticalOffset + (24.0f * ShooterConstants::InchesToMeters);
    // float VertexXPose = dx - (24.0755062252f * ShooterConstants::InchesToMeters);

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
    float dx = 0.0254f + std::sqrt(std::pow(horizontalOffset, 2.0f) + std::pow(yOffset, 2.0f));

    // table
    struct Entry { float dx; float effectiveRPM; };
    static constexpr std::array<Entry, 8> kTable = {{
    { 0.9398f, 1120.0f },  // 37 inches
    { 1.27f,   1100.0f },  // 50 inches
    { 1.905f,  1160.0f },  // 75 inches
    { 2.54f,   1240.0f },  // 100 inches
    { 3.175f,  1320.0f },  // 125 inches
    { 3.81f,   1398.0f },  // 150 inches
    { 4.445f,  1510.0f },  // 175 inches
    { 5.08f,   1545.0f },  // 200 inches
    }};

    if (dx < kTable.front().dx) return 0.0f;
    if (dx >= kTable.back().dx) return kTable.back().effectiveRPM;

    // i + 1 for no crashing and comparison
    for (size_t i = 0; i + 1 < kTable.size(); i++) {

        // andy's linear interpolation suggestion
        if (dx >= kTable[i].dx && dx < kTable[i + 1].dx) {
            float t = (dx - kTable[i].dx) / (kTable[i + 1].dx - kTable[i].dx);
            float effRPM = kTable[i].effectiveRPM + t * (kTable[i + 1].effectiveRPM - kTable[i].effectiveRPM);
            return effRPM;
        }

    }
    return 0.0f;
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

void Shooter::setManualHoodPosition(float targetAngleDeg) {
    // targetAngleDeg is a hood angle in degrees (same unit as findHoodAngle())
    // deltaDeg is relative to the resting/max angle (68°), matching setHoodPosition's convention
    float deltaDeg = targetAngleDeg - 68.0f;
    double motorTurnsTarget = (deltaDeg / 360.0) * ShooterConstants::motorGearRatio;
    double targetAbsLocal = hoodCenterRot + motorTurnsTarget;

    targetAbs = targetAbsLocal;

    RackMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{units::angle::turn_t{targetAbs}});
}

double Shooter::findHoodAngle() {
    // Inverse of setHoodPosition's motor-turns formula: angle = turns/gearRatio * 360 + restingAngle
    return (RackMotor.GetPosition().GetValueAsDouble() / ShooterConstants::motorGearRatio) * 360.0 + 68.0;
}