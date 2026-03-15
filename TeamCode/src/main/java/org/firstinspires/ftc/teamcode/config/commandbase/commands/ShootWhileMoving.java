package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * ShootWhileMoving — fires balls while the robot is still driving.
 *
 * DriveToPose and Shoot run concurrently. isFinished() waits for both
 * the path AND all balls to be fired before ending.
 *
 * Shoot uses velocityFF=true so the flywheel compensates for the robot
 * moving toward or away from the goal while firing.
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *   new ShootWhileMoving(follower, Poses.SCORE_CLOSE, 3, 1850, isBlue, FiringMode.RAPID, HeadingMode.LINEAR)
 *   new ShootWhileMoving(follower, Poses.SCORE_CLOSE, 3, 1850, isBlue, FiringMode.RAPID, myPiecewiseHeading)
 *   new ShootWhileMoving(follower, new Pose[]{endpoint, ctrl1}, 3, 1850, isBlue, FiringMode.RAPID, HeadingMode.TANGENTIAL)
 *   new ShootWhileMoving(follower, new Pose[]{endpoint, ctrl1}, 3, 1850, isBlue, FiringMode.RAPID, myPiecewiseHeading)
 */
public class ShootWhileMoving extends CommandBase {

    private final DriveToPose drive;
    private final Shoot       shoot;

    /** Straight line, heading mode. */
    public ShootWhileMoving(Follower follower, Pose targetPose,
                            int ballsToFire, double overrideVelocity,
                            boolean isBlue, Shoot.FiringMode firingMode,
                            DriveToPose.HeadingMode headingMode) {
        this.drive = new DriveToPose(follower, targetPose, headingMode);
        this.shoot = new Shoot(follower, ballsToFire, overrideVelocity, isBlue, firingMode, true);
    }

    /** Straight line, piecewise heading. */
    public ShootWhileMoving(Follower follower, Pose targetPose,
                            int ballsToFire, double overrideVelocity,
                            boolean isBlue, Shoot.FiringMode firingMode,
                            PiecewiseHeading piecewiseHeading) {
        this.drive = new DriveToPose(follower, new Pose[]{targetPose}, piecewiseHeading, 1.0);
        this.shoot = new Shoot(follower, ballsToFire, overrideVelocity, isBlue, firingMode, true);
    }

    /** Bezier curve, heading mode. */
    public ShootWhileMoving(Follower follower, Pose[] waypoints,
                            int ballsToFire, double overrideVelocity,
                            boolean isBlue, Shoot.FiringMode firingMode,
                            DriveToPose.HeadingMode headingMode) {
        this.drive = new DriveToPose(follower, waypoints, headingMode);
        this.shoot = new Shoot(follower, ballsToFire, overrideVelocity, isBlue, firingMode, true);
    }

    /** Bezier curve, piecewise heading. */
    public ShootWhileMoving(Follower follower, Pose[] waypoints,
                            int ballsToFire, double overrideVelocity,
                            boolean isBlue, Shoot.FiringMode firingMode,
                            PiecewiseHeading piecewiseHeading) {
        this.drive = new DriveToPose(follower, waypoints, piecewiseHeading, 1.0);
        this.shoot = new Shoot(follower, ballsToFire, overrideVelocity, isBlue, firingMode, true);
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        drive.initialize();
        shoot.initialize();
    }

    @Override
    public void execute() {
        drive.execute();
        shoot.execute();
    }

    @Override
    public boolean isFinished() {
        return drive.isFinished() && shoot.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        drive.end(interrupted);
        shoot.end(interrupted);
    }
}