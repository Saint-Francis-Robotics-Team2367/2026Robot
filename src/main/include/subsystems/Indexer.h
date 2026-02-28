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


constexpr int motorID = 13;//Have to change this later

class Indexer : public frc2::Subsystem {
    public:
        Indexer();
        frc2::CommandPtr index();
        void init();
        void setIndexerSpeed(double speed);
        void stopIndexer();
       
       
    private:
        ctre::phoenix6::hardware::TalonFX indexerMotor{motorID};
        ctre::phoenix6::configs::TalonFXConfiguration indexerConfigs{};
};
