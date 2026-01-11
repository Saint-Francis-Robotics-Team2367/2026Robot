#pragma once

#include <rev/SparkMax.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/config/SparkMaxConfig.h>
#include <array>

#define MotorID1 10
#define MotorID2 11

class Climber {
    public:
        //Rotation Setpoints
        double stowedPosition = 0.0;
        double climbPosition = 100.0;

        std::array<double, 2> climberState{stowedPosition, climbPosition};

        int setpointState;

        void init();
        void setState(double setpointState);
        void climb();
        void stop();

        rev::spark::SparkMax motor1 = rev::spark::SparkMax(MotorID1, rev::spark::SparkLowLevel::MotorType::kBrushless);
        rev::spark::SparkRelativeEncoder enc1 = motor1.GetEncoder();
        rev::spark::SparkMaxConfig config{};
        rev::spark::SparkClosedLoopController motorctr1 = motor1.GetClosedLoopController();

        rev::spark::SparkMax motor2 = rev::spark::SparkMax(MotorID2, rev::spark::SparkLowLevel::MotorType::kBrushless);
        rev::spark::SparkRelativeEncoder enc2 = motor2.GetEncoder();
        rev::spark::SparkMaxConfig config2{};
        rev::spark::SparkClosedLoopController motorctr2 = motor2.GetClosedLoopController();
};