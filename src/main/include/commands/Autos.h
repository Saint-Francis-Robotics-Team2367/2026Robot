#pragma once

#include <frc2/command/CommandPtr.h>

class Shooter;
class Spindexer;
class DeployIntake;
class RunIntake;

namespace autos {

void RegisterNamedCommands(
    Shooter* shooter,
    Spindexer* spindexer,
    DeployIntake* deployIntake,
    RunIntake* runIntake);

frc2::CommandPtr SpinUpShooter(Shooter* shooter);
frc2::CommandPtr Shoot(
    Shooter* shooter, Spindexer* spindexer, DeployIntake* deployIntake);
frc2::CommandPtr StopShooter(
    Shooter* shooter, Spindexer* spindexer, DeployIntake* deployIntake);

frc2::CommandPtr DeployIntakeCommand(DeployIntake* deployIntake);
frc2::CommandPtr RunIntakeCommand(RunIntake* runIntakeSubsystem);
frc2::CommandPtr StopIntakeCommand(
    RunIntake* runIntakeSubsystem, DeployIntake* deployIntakeSubsystem);

frc2::CommandPtr FeedHopper(Spindexer* spindexer);
frc2::CommandPtr ReverseHopper(Spindexer* spindexer);
frc2::CommandPtr StopHopper(Spindexer* spindexer);

}  // namespace autos
