// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/path/PathPlannerPath.h"

frc2::CommandPtr autos::FollowPath(DriveSubsystem* subsystem, std::string pathName1, std::string pathName2, std::string pathName3) {
    auto path1 = pathplanner::PathPlannerPath::fromPathFile(pathName1);
    auto path2 = pathplanner::PathPlannerPath::fromPathFile(pathName2);
    auto path3 = pathplanner::PathPlannerPath::fromPathFile(pathName3);

    // auto resetPose = frc2::cmd::RunOnce(
    //     [subsystem, &path1] {
    //         subsystem->resetOdometry(path1.get()->getStartingHolonomicPose().value());
    //     }
    // );

    return frc2::cmd::Sequence(
        // std::move(resetPose),
        pathplanner::AutoBuilder::followPath(path1)
        // frc2::cmd::Wait(1_s),
        // pathplanner::AutoBuilder::followPath(path2)
    );
}
