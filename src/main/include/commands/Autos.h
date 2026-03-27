// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/Shooter.h"
#include "subsystems/vision/Limelight.h"
#include "subsystems/Spindexer.h"
#include "subsystems/Intake/DeployIntake.h"

namespace autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr FollowPath(DriveSubsystem* subsystem, Shooter* mShooter, Limelight* cam, Spindexer* mSpindexer, DeployIntake* mDeployIntake, std::string pathName1);
}  // namespace autos
