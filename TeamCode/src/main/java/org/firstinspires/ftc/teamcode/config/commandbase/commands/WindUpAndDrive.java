package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * WindUpAndDrive — drives a path while spinning the flywheel up to speed.
 * The conveyor stays closed the entire time — no feeding happens here.
 *
 * All pathing logic lives in DriveToPose. To change heading, speed, or waypoints,
 * change DriveToPose — not this class.
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *   new WindUpAndDrive(follower, Poses.CLOSE_TOSCORE, 1850, HeadingMode.LINEAR, 1.0)
 *   new WindUpAndDrive(follower, Poses.CLOSE_TOSCORE, 1850, myPiecewiseHeading, 1.0)
 *   new WindUpAndDrive(follower, new Pose[]{endpoint, ctrl1}, 1850, HeadingMode.TANGENTIAL, 1.0)
 *   new WindUpAndDrive(follower, new Pose[]{endpoint, ctrl1}, 1850, myPiecewiseHeading, 1.0)
 */
public class WindUpAndDrive extends CommandBase {

    private final DriveToPose drive;
    private final double      flywheelVelocity;
    private final Robot       robot = Robot.getInstance();

    /** Straight line, heading mode. */
    public WindUpAndDrive(Follower follower, Pose targetPose, double flywheelVelocity,
                          DriveToPose.HeadingMode headingMode, double driveSpeed) {
        this.drive            = new DriveToPose(follower, targetPose, headingMode, driveSpeed);
        this.flywheelVelocity = flywheelVelocity;
        addRequirements(robot.flywheel, robot.conveyor);
    }

    /** Straight line, piecewise heading. */
    public WindUpAndDrive(Follower follower, Pose targetPose, double flywheelVelocity,
                          PiecewiseHeading piecewiseHeading, double driveSpeed) {
        this.drive            = new DriveToPose(follower, new Pose[]{targetPose}, piecewiseHeading, driveSpeed);
        this.flywheelVelocity = flywheelVelocity;
        addRequirements(robot.flywheel, robot.conveyor);
    }

    /** Bezier curve, heading mode. */
    public WindUpAndDrive(Follower follower, Pose[] waypoints, double flywheelVelocity,
                          DriveToPose.HeadingMode headingMode, double driveSpeed) {
        this.drive            = new DriveToPose(follower, waypoints, headingMode, driveSpeed);
        this.flywheelVelocity = flywheelVelocity;
        addRequirements(robot.flywheel, robot.conveyor);
    }

    /** Bezier curve, piecewise heading. */
    public WindUpAndDrive(Follower follower, Pose[] waypoints, double flywheelVelocity,
                          PiecewiseHeading piecewiseHeading, double driveSpeed) {
        this.drive            = new DriveToPose(follower, waypoints, piecewiseHeading, driveSpeed);
        this.flywheelVelocity = flywheelVelocity;
        addRequirements(robot.flywheel, robot.conveyor);
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        robot.flywheel.setVelocity(flywheelVelocity);
        drive.initialize();
    }

    @Override
    public void execute() {
        robot.conveyor.stop();
        robot.flywheel.setVelocity(flywheelVelocity);
        drive.execute();
    }

    @Override
    public boolean isFinished() { return drive.isFinished(); }

    @Override
    public void end(boolean interrupted) {
        drive.end(interrupted);
        // Flywheel stays running — Shoot takes over
    }
}