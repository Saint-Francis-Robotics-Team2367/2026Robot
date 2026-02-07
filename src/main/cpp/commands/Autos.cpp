// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/path/PathPlannerPath.h"

frc2::CommandPtr autos::FollowPath(DriveSubsystem* subsystem, std::string pathName) {
    auto path = pathplanner::PathPlannerPath::fromPathFile(pathName);
    return pathplanner::AutoBuilder::followPath(path);

    // return frc2::cmd::Sequence(subsystem->Drive(),
    //                          ExampleCommand(subsystem).ToPtr());
}
