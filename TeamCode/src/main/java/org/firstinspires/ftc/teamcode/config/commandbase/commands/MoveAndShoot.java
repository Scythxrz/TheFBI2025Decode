package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

/**
 * MoveAndShoot — drives to a pose, then fires after arriving.
 *
 * Composed of WindUpAndDrive → Shoot in sequence.
 * The flywheel spins up during the drive so it's ready the moment the robot arrives.
 * To change pathing, change DriveToPose/WindUpAndDrive. To change shooting, change Shoot.
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *   new MoveAndShoot(follower, Poses.SCORE_CLOSE, 3, 1850, isBlue, FiringMode.RAPID, HeadingMode.LINEAR)
 *   new MoveAndShoot(follower, Poses.SCORE_CLOSE, 3, 1850, isBlue, FiringMode.RAPID, myPiecewiseHeading)
 *   new MoveAndShoot(follower, new Pose[]{endpoint, ctrl1}, 3, 1850, isBlue, FiringMode.RAPID, HeadingMode.TANGENTIAL)
 *   new MoveAndShoot(follower, new Pose[]{endpoint, ctrl1}, 3, 1850, isBlue, FiringMode.RAPID, myPiecewiseHeading)
 */
public class MoveAndShoot extends SequentialCommandGroup {

    /** Straight line, heading mode. */
    public MoveAndShoot(Follower follower, Pose targetPose,
                        int ballsToFire, double overrideVelocity,
                        boolean isBlue, Shoot.FiringMode firingMode,
                        DriveToPose.HeadingMode headingMode) {
        addCommands(
                new WindUpAndDrive(follower, targetPose, overrideVelocity, headingMode, 1.0),
                new Shoot(follower, ballsToFire, overrideVelocity, isBlue, firingMode)
        );
    }

    /** Straight line, piecewise heading. */
    public MoveAndShoot(Follower follower, Pose targetPose,
                        int ballsToFire, double overrideVelocity,
                        boolean isBlue, Shoot.FiringMode firingMode,
                        PiecewiseHeading piecewiseHeading) {
        addCommands(
                new WindUpAndDrive(follower, targetPose, overrideVelocity, piecewiseHeading, 1.0),
                new Shoot(follower, ballsToFire, overrideVelocity, isBlue, firingMode)
        );
    }

    /** Bezier curve, heading mode. */
    public MoveAndShoot(Follower follower, Pose[] waypoints,
                        int ballsToFire, double overrideVelocity,
                        boolean isBlue, Shoot.FiringMode firingMode,
                        DriveToPose.HeadingMode headingMode) {
        addCommands(
                new WindUpAndDrive(follower, waypoints, overrideVelocity, headingMode, 1.0),
                new Shoot(follower, ballsToFire, overrideVelocity, isBlue, firingMode)
        );
    }

    /** Bezier curve, piecewise heading. */
    public MoveAndShoot(Follower follower, Pose[] waypoints,
                        int ballsToFire, double overrideVelocity,
                        boolean isBlue, Shoot.FiringMode firingMode,
                        PiecewiseHeading piecewiseHeading) {
        addCommands(
                new WindUpAndDrive(follower, waypoints, overrideVelocity, piecewiseHeading, 1.0),
                new Shoot(follower, ballsToFire, overrideVelocity, isBlue, firingMode)
        );
    }
    // ─── Enum converters ─────────────────────────────────────────────────────

}