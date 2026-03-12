#pragma once


#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>


#include <frc2/command/Subsystem.h>
#include "frc2/command/StartEndCommand.h"
#include "frc2/command/SubsystemBase.h"
#include "frc2/command/Requirements.h"
#include "frc2/command/Commands.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/button/Trigger.h"

#include <Constants.h>

class Indexer : public frc2::Subsystem {

    public:
        void init();
        void setIndexerSpeed(double speed);
        void stopIndexer();
        frc2::CommandPtr RunIndexer(Indexer* indexer, double speed);
        bool IndexerStall();
       
    private:
        ctre::phoenix6::hardware::TalonFX indexerMotor{IndexerConstants::IndexerMotorID, "Drivetrain"}; // Use CANivore bus if applicable
        ctre::phoenix6::configs::TalonFXConfiguration indexerConfigs;
};