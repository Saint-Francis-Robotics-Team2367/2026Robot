#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace pho = ctre::phoenix6;

class TalonFXMotor
{
public:
    enum controlMode
    {
        VELOCITY,
        POSITION,
        OPENLOOP
    };

private:
    double lastInput = NAN;
    controlMode lastMode;
    pho::controls::VelocityVoltage m_voltageVelocity = pho::controls::VelocityVoltage{0_tps}; // {0_tps, 0_tr_per_s_sq, true, 0_V, 0, false}
    pho::controls::PositionVoltage m_voltagePosition = pho::controls::PositionVoltage{0_tr}; // {0_tr, 0_tps, true, 0_V, 0, false}

    pho::configs::TalonFXConfiguration currentConfiguration;

    void modeSet(controlMode mode, double input)
    {
        switch (mode)
        {
        case POSITION:
            motor.SetControl(m_voltagePosition.WithPosition(units::angle::turn_t{input}).WithSlot(POSITION));
            break;
        case VELOCITY:
            motor.SetControl(m_voltageVelocity.WithVelocity((input / 60.0) * 1_tps).WithSlot(VELOCITY));
            break;
        case OPENLOOP:
            motor.Set(input);
            break;
        }
    }

public:
    pho::hardware::TalonFX motor;

    TalonFXMotor(int deviceID) : motor(pho::hardware::TalonFX(deviceID, "rio"))
    {
    }

    pho::configs::TalonFXConfiguration getConfig()
    {
        return currentConfiguration;
    }

    void init()
    {

        pho::configs::TalonFXConfiguration configs{};

        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        configs.Slot0.kP = 0.11;   // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.0;    // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.0001 volts output
        configs.Slot0.kV = 0.0;   // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        configs.Slot0.kS = 0.00;

        configs.TorqueCurrent.PeakForwardTorqueCurrent = units::ampere_t(10);
        configs.TorqueCurrent.PeakReverseTorqueCurrent = units::ampere_t(-10);

        /* Percent supply gains when we get a Slot 2 */
        configs.Slot1.kP = 3.5;    // An error of 100 rotations per second results in 100% output
        configs.Slot1.kI = 0.0;    // An error of 1 rotation per second increases output by 0.04V every second
        configs.Slot1.kD = 0.1; // A change of 1 rotation per second squared results in 0.00001 volts output
        configs.Slot1.kV = 0.013;   // Approximately 1.3% for each rotation per second

        configs.MotorOutput.NeutralMode = pho::signals::NeutralModeValue::Brake;
        configs.MotorOutput.Inverted = pho::signals::InvertedValue::Clockwise_Positive;

        configs.MotorOutput.PeakForwardDutyCycle = 1.0;  
        configs.MotorOutput.PeakReverseDutyCycle = -1.0;
        motor.GetConfigurator().Apply(configs);
        currentConfiguration = configs;
    }
    /**
     * Control Modes:
     * VELOCITY: Run to motor to a certain RPM
     * POSITION: Run the motor a number of rotations
     * OPENLOOP: Run the motor on a duty cycle -1 to 1
     */
    void set(controlMode mode, double input)
    {
        if (input != lastInput || mode != lastMode)
        {
            modeSet(mode, input);
        }
        lastInput = input;
        lastMode = mode;
    }

    double getVelocity() {
        return motor.GetRotorVelocity().GetValueAsDouble() * 60.0;
    }

    double getPosition() {
        return motor.GetRotorPosition().GetValueAsDouble();
    }

    /**
     * Do not run in periodic functions
     * Applying a motor configuration takes up to 5 ms
     * This should only be done in RobotInit or constructors
    */
    void setInvert(pho::signals::InvertedValue inversionValue) 
    {
        currentConfiguration.MotorOutput.Inverted = inversionValue;
        motor.GetConfigurator().Apply(currentConfiguration);
    }

};