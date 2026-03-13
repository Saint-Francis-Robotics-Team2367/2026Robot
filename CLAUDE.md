# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Deploy

```bash
# Build for RoboRIO
./gradlew build

# Deploy to robot (robot must be connected)
./gradlew deploy

# Build + deploy in one step
./gradlew deployfrcUserProgram

# Run tests
./gradlew test
```

This is a **C++ FRC robot project** using GradleRIO 2026.1.1. Source files live in `src/main/cpp/`, headers in `src/main/include/`. Files placed in `src/main/deploy/` are copied to `/home/lvuser/deploy/` on the RoboRIO at deploy time.

## Architecture

**Command-based WPILib** with a `frc::TimedRobot` at the top. `Robot::RobotPeriodic()` runs the command scheduler and calls manual `periodic()` methods on non-subsystem singletons.

### Key classes

| Class | Location | Role |
|---|---|---|
| `RobotContainer` | `src/main/cpp/RobotContainer.cpp` | Instantiates all subsystems, wires controller bindings |
| `DriveSubsystem` | `subsystems/DriveSubsystem` | 4-wheel swerve drive with Pigeon2 gyro, `SwerveDrivePoseEstimator` |
| `QuestNav` | `subsystems/vision/QuestNav.h` | Singleton; reads Meta Quest 3 pose via NetworkTables + protobufs |
| `PhotonVision` | `subsystems/vision/PhotonVision` | PhotonLib wrapper for Limelight3 ("Lemonlight", camera "Meyer") |
| `QuestNavCalibration` | `subsystems/vision/QuestNavCalibration.h` | Header-only offset object; converts QuestNav-relative poses to field-absolute using a PhotonVision snapshot |
| `Turret` | `subsystems/Turret` | Single-axis TalonFX turret with auto-targeting toward goal hub |
| `HoodedShooter` | `subsystems/Shooter` | Flywheel + rack/hood angle, calculates RPM from QuestNav pose |

### Pose pipeline

1. `QuestNav` provides a *relative* pose (origin = wherever the Quest was when it booted).
2. At startup `PhotonVision::periodic()` waits for a confident AprilTag estimate from the field layout (`FRC2026_WELDED.fmap`, deployed to `/home/lvuser/deploy/`).
3. On the first confident estimate it calls `QuestNavCalibration::calibrate(photonPose, questPose)`.
4. `QuestNavCalibration::getCorrectedPose(questPose)` converts any future QuestNav pose to a field-absolute pose using `Pose2d::RelativeTo` + `TransformBy` — this corrects both translation and rotation frame differences.
5. `Robot::RobotPeriodic()` seeds drivetrain odometry once after calibration via `resetOdometry()`.

### Vendordeps

| Library | File |
|---|---|
| CTRE Phoenix 6 v26.1.1 | `vendordeps/Phoenix6-26.1.1.json` |
| WPILib New Commands | `vendordeps/WPILibNewCommands.json` |
| PhotonLib v2026.3.1 | `vendordeps/photonlib.json` |

### Hardware IDs (CAN bus "Drivetrain")

Defined in `Constants.h`. Drivetrain: motors 1–11, encoders 2–12, Pigeon2 ID 0. Shooter: 36 (flywheel), 27 (rack), 21 (feeder). Turret: motor 18, encoder 60. Indexer: 55. Intake pivot: 13, roller: 14, hopper: 16.

### PhotonVision setup notes

- Device hostname: `lemonlight` (Limelight3 running PhotonVision)
- Camera name configured in PhotonVision UI: `Meyer`
- **`kRobotToCamera` in `PhotonVision.cpp` is a placeholder** — measure the physical camera mount position and update before competition.
- The `.fmap` field layout file must be placed at `src/main/deploy/FRC2026_WELDED.fmap`; it is not bundled in the repo yet.
- PhotonVision version on the coprocessor must match `vendordeps/photonlib.json` (v2026.3.1).
