#include "commands/Autos.h"

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "Constants.h"
#include "subsystems/Shooter.h"
#include "subsystems/Spindexer.h"
#include "subsystems/Intake/DeployIntake.h"
#include "subsystems/Intake/RunIntake.h"

using namespace pathplanner;

namespace autos {

void RegisterNamedCommands(
    Shooter* shooter,
    Spindexer* spindexer,
    DeployIntake* deployIntake,
    RunIntake* runIntake) {
  NamedCommands::registerCommand("SpinUpShooter", SpinUpShooter(shooter));
  NamedCommands::registerCommand(
      "Shoot", Shoot(shooter, spindexer, deployIntake));
  NamedCommands::registerCommand(
      "StopShooter", StopShooter(shooter, spindexer, deployIntake));

  NamedCommands::registerCommand("DeployIntake", DeployIntakeCommand(deployIntake));
  NamedCommands::registerCommand("RunIntake", RunIntakeCommand(runIntake));
  NamedCommands::registerCommand(
      "StopIntake", StopIntakeCommand(runIntake, deployIntake));

  NamedCommands::registerCommand("FeedHopper", FeedHopper(spindexer));
  NamedCommands::registerCommand("ReverseHopper", ReverseHopper(spindexer));
  NamedCommands::registerCommand("StopHopper", StopHopper(spindexer));
}

frc2::CommandPtr SpinUpShooter(Shooter* shooter) {
  auto spinUp = shooter->RunOnce(
      [shooter] {
        shooter->setManualHoodPosition(AutoConstants::kAutoHoodAngle);
        shooter->setFlywheelSpeed(-AutoConstants::kAutoFlywheelRPM);
      });

  // In desktop sim the flywheel never reaches target RPM — don't block the auto.
  if (frc::RobotBase::IsSimulation()) {
    return frc2::cmd::Sequence(std::move(spinUp), frc2::cmd::Wait(0.5_s));
  }

  return frc2::cmd::Sequence(
      std::move(spinUp),
      frc2::cmd::WaitUntil(
          [shooter] {
            return shooter->getShooterVelocity() >
                   (0.95 * (1 / ShooterConstants::SHOOTEREFFICIENCY) *
                    AutoConstants::kAutoFlywheelRPM);
          }));
}

frc2::CommandPtr Shoot(
    Shooter* shooter, Spindexer* spindexer, DeployIntake* deployIntake) {
  // Instant start — auto Wait + StopShooter control how long feeding runs.
  return frc2::cmd::Parallel(
      spindexer->RunOnce(
          [spindexer] {
            spindexer->setSpindexerSpeed(AutoConstants::kFeedSpindexerRPM);
          }),
      deployIntake->RunOnce(
          [deployIntake] { deployIntake->shooterDeploy(); }),
      shooter->RunOnce(
          [shooter] {
            shooter->setFlywheelSpeed(-AutoConstants::kAutoFlywheelRPM);
          }));
}

frc2::CommandPtr StopShooter(
    Shooter* shooter, Spindexer* spindexer, DeployIntake* deployIntake) {
  return frc2::cmd::Parallel(
      shooter->RunOnce(
          [shooter] {
            shooter->ShooterMotor.SetControl(
                ctre::phoenix6::controls::DutyCycleOut{0.0});
            shooter->moveHoodToZero();
          }),
      frc2::cmd::RunOnce([spindexer] { spindexer->stopSpindexer(); }),
      deployIntake->RunOnce([deployIntake] { deployIntake->shooterRetract(); }));
}

frc2::CommandPtr DeployIntakeCommand(DeployIntake* deployIntake) {
  return deployIntake->RunOnce(
      [deployIntake] {
        deployIntake->deploy();
        deployIntake->deployed = true;
      });
}

frc2::CommandPtr RunIntakeCommand(RunIntake* runIntakeSubsystem) {
  // Instant start — keeps rollers running until StopIntake. A StartEnd would
  // block PathPlanner sequential autos forever after the first path.
  return runIntakeSubsystem->RunOnce(
      [runIntakeSubsystem] {
        runIntakeSubsystem->setMotorSpeed(AutoConstants::kIntakeRollerRPM);
      });
}

frc2::CommandPtr StopIntakeCommand(
    RunIntake* runIntakeSubsystem, DeployIntake* deployIntakeSubsystem) {
  return frc2::cmd::Parallel(
      runIntakeSubsystem->RunOnce(
          [runIntakeSubsystem] { runIntakeSubsystem->stop(); }),
      deployIntakeSubsystem->RunOnce(
          [deployIntakeSubsystem] {
            deployIntakeSubsystem->retract();
            deployIntakeSubsystem->deployed = false;
          }));
}

frc2::CommandPtr FeedHopper(Spindexer* spindexer) {
  return spindexer->RunOnce(
      [spindexer] {
        spindexer->setSpindexerSpeed(AutoConstants::kFeedSpindexerRPM);
      });
}

frc2::CommandPtr ReverseHopper(Spindexer* spindexer) {
  return spindexer->RunOnce(
      [spindexer] {
        spindexer->setSpindexerSpeed(AutoConstants::kReverseSpindexerRPM);
      });
}

frc2::CommandPtr StopHopper(Spindexer* spindexer) {
  return frc2::cmd::RunOnce([spindexer] { spindexer->stopSpindexer(); });
}

}  // namespace autos
