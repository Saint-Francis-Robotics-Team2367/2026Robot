#include "Shooter.h"
using namespace std;


const float GRAVITY = 9.81;
const float SHOOTERWHEELDIAMETER = 0.1;
const float PI = M_PI;


void Shooter::stop() {
    // Stop all motors
    ShooterMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
    RackMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
}


void Shooter::init() {
    // Reset all motors to factory defaults
    ShooterMotor.GetConfigurator().Apply(FlywheelConfig);
    RackMotor.GetConfigurator().Apply(RackConfig);


    // Configure PID constants for Flywheel Master (Velocity Control)
    FlywheelConfig.Slot0.kP = FlywheelP;
    FlywheelConfig.Slot0.kI = FlywheelI;
    FlywheelConfig.Slot0.kD = FlywheelD;
    FlywheelConfig.Slot0.kV = FlywheelV;


    // Configure PID constants for Rack Motor (Position Control)
    RackConfig.Slot0.kP = RackP;
    RackConfig.Slot0.kI = RackI;
    RackConfig.Slot0.kD = RackD;
    RackConfig.Slot0.kG = RackG;


    // Apply configurations
    ShooterMotor.GetConfigurator().Apply(FlywheelConfig, kTimeoutMs);
    RackMotor.GetConfigurator().Apply(RackConfig, kTimeoutMs);
}


bool Shooter::setFlywheelSpeed(float shooterRPM) {
    // set shooter velocity
    ShooterMotor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{units::angular_velocity::turns_per_second_t{shooterRPM / 60.0}});
    std::this_thread::sleep_for(std::chrono::seconds(5));

    double targetVelocity = shooterRPM / 60.0;
    double actualVelocity = ShooterMotor.GetVelocity().GetValue().value();
    double tolerance = 0.05; // or whatever tolerance you want

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
void Shooter::setHoodPosition(float shooterRPM, float horizontalOffset, float yOffset, float cameraHeight, float initialAngle, float maxAngle, float GearRatio) {


    // Convert shooter RPM to linear velocity (m/s)
    float flywheelCircumference = PI * SHOOTERWHEELDIAMETER;
    float shooterVelocity = (shooterRPM * flywheelCircumference) / 60.0f;
    float exitVelo = 0.95f * shooterVelocity;


    // Target point (dx, dy) in meters
    float dx = std::sqrt(std::pow(horizontalOffset * 0.0254, 2.0) + std::pow(yOffset * 0.0254f, 2.0f));
    const float dy = 72.0 * 0.0254;
    const float verticalOffset = dy - (cameraHeight * 0.0254);


    // Desired vertex location
    const float VertexYPose = verticalOffset + (6.0f * 0.0254f);
    float VertexXPose = dx - (24.0755062252 * 0.0254);


    // Solve projectile equation at (dx, dy)
    const float A = (GRAVITY * dx * dx) / (2.0 * exitVelo * exitVelo);
    const float discriminant = dx * dx - 4.0 * A * (A + dy);
    float unknownAngle = initialAngle * (PI / 180.0);


    // check for validity/ensure we pick the right curve
    if (discriminant >= 0.0f && exitVelo > 0.0f && dx > 0.0f) {
        const float sqrtDisc = std::sqrt(discriminant);


        const float t1 = (dx - sqrtDisc) / (2.0f * A);
        const float t2 = (dx + sqrtDisc) / (2.0f * A);


        const float theta1 = atan(t1);
        const float theta2 = atan(t2);


        auto apexError = [&](float theta) {
            float s = sin(theta);
            float c = cos(theta);


            float xApex = (exitVelo * exitVelo * s * c) / GRAVITY;
            float yApex = (exitVelo * exitVelo * s * s) / (2.0f * GRAVITY);


            float ex = xApex - VertexXPose;
            float ey = yApex - VertexYPose;


            return ex * ex + ey * ey;
        };


        bool v1 = theta1 > 0.0 && theta1 < (PI * 0.5);
        bool v2 = theta2 > 0.0 && theta2 < (PI * 0.5);


        if (v1 && v2)
            unknownAngle = (apexError(theta1) <= apexError(theta2)) ? theta1 : theta2;
        else if (v1)
            unknownAngle = theta1;
        else if (v2)
            unknownAngle = theta2;
    }


    float hoodAngleDegrees = unknownAngle * 180.0 / PI;
    float requiredAngle = hoodAngleDegrees - initialAngle;
    double currentDeg = RackEncoder.GetAbsolutePosition().GetValue().value() * 360.0;
    if (std::fabs(requiredAngle - currentDeg) > 0.25 && hoodAngleDegrees >= initialAngle && hoodAngleDegrees <= maxAngle) {
        RackMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{units::angle::turn_t{GearRatio * (requiredAngle / 360.0)}});
        hoodAngleDegrees = initialAngle;
        std::cout << "Hood Angle Set To: " << hoodAngleDegrees << " degrees" << std::endl;
    } else {
        std::cout << "Hood Angle Error : " << hoodAngleDegrees << " degrees remains" << std::endl;
    }
}
