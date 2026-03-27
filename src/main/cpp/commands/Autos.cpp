// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/path/PathPlannerPath.h"

frc2::CommandPtr autos::FollowPath(DriveSubsystem* subsystem, Shooter* mShooter, Limelight* cam, Spindexer* mSpindexer, DeployIntake* mDeployIntake, std::string pathName1) {
    auto path1 = pathplanner::PathPlannerPath::fromPathFile(pathName1);

    return frc2::cmd::Sequence(
        pathplanner::AutoBuilder::followPath(path1),
        frc2::cmd::Wait(0.25_s),
        frc2::cmd::RunOnce(
            [mShooter, cam] {
                mShooter->distanceToTag = std::cos((cam->ty + 15.0) * (std::numbers::pi / 180.0)) * cam->distanceToTag;
                mShooter->xOffset = std::sin(cam->tx * (std::numbers::pi / 180.0)) * mShooter->distanceToTag;
                mShooter->yOffset = std::cos(cam->tx * (std::numbers::pi / 180.0)) * mShooter->distanceToTag;
                mShooter->optimalRPM = mShooter->findOptimalRPM();
                mShooter->setHoodPosition(mShooter->optimalRPM, mShooter->xOffset, mShooter->yOffset);

                frc::SmartDashboard::PutNumber("Shooter Distance (dx)", mShooter->xOffset);
                frc::SmartDashboard::PutNumber("Shooter Distance (dy)", mShooter->yOffset);
            }
        ),
        frc2::cmd::Parallel(
            frc2::cmd::StartEnd(
                [mShooter] {
                    mShooter->setFlywheelSpeed(-mShooter->optimalRPM);
                },
                [mShooter] {
                    mShooter->ShooterMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
                    mShooter->moveHoodToZero();
                }
            ),
            frc2::cmd::Sequence(
                frc2::cmd::WaitUntil(
                    [mShooter, cam] {
                        return cam->hasTarget && 
                            (mShooter->getShooterVelocity() > 
                                (0.95 * (1.0 / ShooterConstants::SHOOTEREFFICIENCY) * 
                                mShooter->optimalRPM));
                    }
                ),
                frc2::cmd::RunOnce([] {
                    frc::SmartDashboard::PutString("Ran", "RAN INDEXER AND FEEDER");
                }),
                frc2::cmd::Parallel(
                    mSpindexer->RunSpindexer(mSpindexer, -6250),
                    mDeployIntake->masterIntakeCommand(mDeployIntake, true)
                )
            )
        ).WithTimeout(4_s)
    );
}
