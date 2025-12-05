#include "Trajectory.h"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/kinematics/ChassisSpeeds.h>

// controller used to track trajectories + correct minor disturbances
static frc::HolonomicDriveController controller{
    frc::PIDController{6e-4, 0, 0}, // Change PIDs to be more accurate
    frc::PIDController{6e-4, 0, 0}, // Change PIDs to be more accurate
    frc::ProfiledPIDController<units::radian>{
        0.5, 0, 0,
        frc::TrapezoidProfile<units::radian>::Constraints{
            units::radians_per_second_t(5.0), // prev: 5.0
            units::radians_per_second_squared_t(100)}}}; // prev: 100

/**
 * Drives robot to the next state on trajectory
 * Odometry must be in meters
 */
void Trajectory::driveToState(PathPlannerTrajectoryState const &state)
{
    // Calculate new chassis speeds given robot position and next desired state in trajectory
    frc::ChassisSpeeds const correction = controller.Calculate(mDrive.GetPoseEstimatorPose(), frc::Pose2d{state.pose.Translation(), state.heading}, state.linearVelocity, state.heading);

    // Calculate x, y speeds from MPS
    double vy_feet = correction.vx.value() * 3.281;
    double vx_feet = correction.vy.value() * 3.281;
    
    // Clamp rot speed to 2.0 since that is the max rot we allow
    double rot = -std::clamp(correction.omega.value(), -moduleMaxRot, moduleMaxRot);

    frc::SmartDashboard::PutNumber("autoHeading", state.heading.Degrees().value());
    frc::SmartDashboard::PutNumber("auto odometry x", mDrive.GetPoseEstimatorPose().X().value());
    frc::SmartDashboard::PutNumber("autoVY", vy_feet);
    frc::SmartDashboard::PutNumber("autoVX", vx_feet);
    frc::SmartDashboard::PutNumber("autoRot", rot);

    mDrive.Drive(ChassisSpeeds{-vx_feet, vy_feet, rot}, pigeon.getBoundedAngleCCW(), true, true);
}

/**
 * Follows pathplanner trajectory
 */
void Trajectory::follow(std::string const &traj_dir_file_path, bool flipAlliance, bool intake, bool first, float startAngle)
{
    auto path = PathPlannerPath::fromPathFile(traj_dir_file_path);

    // switches path to red alliance (mirrors it)
    if (flipAlliance)
    {
        path = path->flipPath();
    }

    PathPlannerTrajectory traj = PathPlannerTrajectory(path, frc::ChassisSpeeds(), 0_rad, config);

    if (first)
    {
        auto const initialState = traj.getInitialState();
        auto const initialPose = initialState.pose.Translation();

        // set second param to initial holonomic rotation
        // mDrive.resetOdometry(initialPose, units::angle::degree_t(startAngle));
        mDrive.resetPoseEstimator(initialPose, units::angle::degree_t(startAngle));
    }
    if (mDrive.state == DriveState::Auto) {
        frc::Timer trajTimer;
        trajTimer.Start();

        while ((mDrive.state == DriveState::Auto) && (trajTimer.Get() <= traj.getTotalTime()))
        {
            if (intake)
            {
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }

            auto currentTime = trajTimer.Get();
            auto sample = traj.sample(currentTime);

            driveToState(sample);
            mDrive.updatePoseEstimator(cameraFront, false, frc::Timer::GetFPGATimestamp());

            // frc::SmartDashboard::PutNumber("curr pose x meters", mDrive.getOdometryPose().Translation().X().value());
            // frc::SmartDashboard::PutNumber("curr pose y meters", mDrive.getOdometryPose().Translation().Y().value());
            frc::SmartDashboard::PutNumber("curr pose x meters", mDrive.GetPoseEstimatorPose().Translation().X().value());
            frc::SmartDashboard::PutNumber("curr pose y meters", mDrive.GetPoseEstimatorPose().Translation().Y().value());

            using namespace std::chrono_literals;

            // refresh rate of holonomic drive controller's PID controllers (edit if needed)
            double delayStart = frc::Timer::GetFPGATimestamp().value();
            while (mDrive.state == DriveState::Auto && frc::Timer::GetFPGATimestamp().value() - delayStart < 0.02) {
            };
        }
    }
    mDrive.Drive(ChassisSpeeds(0, 0, 0), pigeon.getBoundedAngleCCW(), true, false);
}

void Trajectory::followTeleop(std::shared_ptr<pathplanner::PathPlannerPath> path, bool flipAlliance)
{
    // switches path to red alliance (mirrors it)
    if (flipAlliance)
    {
        path = path->flipPath();
    }

    PathPlannerTrajectory traj = PathPlannerTrajectory(path, frc::ChassisSpeeds(), 0_rad, config);

    if (mDrive.state == DriveState::Teleop) {
        frc::Timer trajTimer;
        trajTimer.Start();

        while ((mDrive.state == DriveState::Teleop) && (trajTimer.Get() <= traj.getTotalTime()))
        {
            auto currentTime = trajTimer.Get();
            auto sample = traj.sample(currentTime);

            driveToState(sample);
            mDrive.updatePoseEstimator(cameraFront, false, frc::Timer::GetFPGATimestamp());

            // frc::SmartDashboard::PutNumber("curr pose x meters", mDrive.getOdometryPose().Translation().X().value());
            // frc::SmartDashboard::PutNumber("curr pose y meters", mDrive.getOdometryPose().Translation().Y().value());
            frc::SmartDashboard::PutNumber("curr pose x meters", mDrive.GetPoseEstimatorPose().Translation().X().value());
            frc::SmartDashboard::PutNumber("curr pose y meters", mDrive.GetPoseEstimatorPose().Translation().Y().value());

            using namespace std::chrono_literals;

            // refresh rate of holonomic drive controller's PID controllers (edit if needed)
            double delayStart = frc::Timer::GetFPGATimestamp().value();
            while (mDrive.state == DriveState::Auto && frc::Timer::GetFPGATimestamp().value() - delayStart < 0.02) {
            };
        }
    }
    mDrive.Drive(ChassisSpeeds(0, 0, 0), pigeon.getBoundedAngleCCW(), true, false);
}

// EDIT LATER 
/**
 * Calls sequences of follow functions for set paths
 */
void Trajectory::followPath(Trajectory::autos autoTrajectory, bool flipAlliance) // true for red alliance
{
    switch (autoTrajectory)
    {
        case DO_NOTHING:
            break;
        case MOVE_STRAIGHT:
            follow ("Move Straight", flipAlliance, false, false); // if not able to see tag use gyro instead
            waitToScore(7);
            break;
        case auto_1A:
            follow("1 to A", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("A to Top Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Top Coral Station to A", flipAlliance, false, false);
            waitToScore(6);
            follow("A to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_1B:
            follow("1 to B", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("B to Top Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Top Coral Station to B", flipAlliance, false, false);
            waitToScore(6);
            follow("B to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_1C:
            follow("1 to C", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Bottom Coral Station to C", flipAlliance, false, false);
            waitToScore(6);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_1D:
            follow("1 to D", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Bottom Coral Station to D", flipAlliance, false, false);
            waitToScore(6);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_1E:
            follow("1 to E", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Bottom Coral Station to E", flipAlliance, false, false);
            waitToScore(6);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_1F:
            follow("1 to F", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Bottom Coral Station to F", flipAlliance, false, false);
            waitToScore(6);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_2A:
            follow("2 to A", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("A to Top Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Top Coral Station to A", flipAlliance, false, false);
            waitToScore(6);
            follow("A to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_2B:
            follow("2 to B", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("B to Top Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Top Coral Station to B", flipAlliance, false, false);
            waitToScore(6);
            follow("B to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_2C:
            follow("2 to C", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(6);
            follow("Bottom Coral Station to C", flipAlliance, false, false);
            waitToScore(6);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_2D:
            follow("2 to D", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Bottom Coral Station to D", flipAlliance, false, false);
            waitToScore(6);
            follow("D to Bottom Coral Stationn", flipAlliance, false, false);
            break;
        case auto_2E:
            follow("2 to E", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Bottom Coral Station to E", flipAlliance, false, false);
            waitToScore(6);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_2F:    
            follow("2 to F", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Bottom Coral Station to F", flipAlliance, false, false);
            waitToScore(6);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_3A:
            follow("3 to A", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("A to Top Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Top Coral Station to A", flipAlliance, false, false);
            waitToScore(6);
            follow("A to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_3B:
            follow("3 to B", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("B to Top Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Top Coral Station to B", flipAlliance, false, false);
            waitToScore(6);
            follow("B to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_3C:
            follow("3 to C", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Bottom Coral Station to C", flipAlliance, false, false);
            waitToScore(6);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_3D:
            follow("3 to D", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Bottom Coral Station to D", flipAlliance, false, false);
            waitToScore(6);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_3E:
            follow("3 to E", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Bottom Coral Station to E", flipAlliance, false, false);
            waitToScore(6);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_3F:
            follow("3 to F", flipAlliance, false, true, 0.0);
            waitToScore(6);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            // waitToScore(7);
            follow("Bottom Coral Station to F", flipAlliance, false, false);
            waitToScore(6);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            break;
    }
}

void Trajectory::waitToScore(int delaySeconds) {
    mDrive.enableModules();
    delayTimer.Start();

    if (cameraFront.isTargetDetected()) {
        cameraChooser = "cameraFront";
    }
    else if (cameraBack.isTargetDetected()) {
        cameraChooser = "cameraBack";
    }

    while ((delayTimer.Get().value() < 4)) {
        if (cameraChooser == "cameraFront") {
            speeds = mAlign.autoAlignPV(cameraFront, 0.275, -0.078);
            mHeadingController.setSetpoint(cameraFront.getAngleSetpoint() - 180);
        }
        else if (cameraChooser == "cameraBack") {
            speeds = mAlign.autoAlignPV(cameraBack, 0.275, 0.078);
            mHeadingController.setSetpoint(cameraBack.getAngleSetpoint() - 180);
        }
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
        double rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
        mDrive.Drive(
            ChassisSpeeds(vx, vy, rot),
            pigeon.getBoundedAngleCCW(),
            false, false);
        mDrive.updateOdometry();
    }

    mDrive.disableModules();
}