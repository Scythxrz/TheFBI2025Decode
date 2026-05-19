package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

/**
 * MoveAndShoot — drives to a pose then fires for feedTimeMs after arriving.
 * WindUpAndDrive spins the flywheel during transit so it is ready on arrival.
 *
 * slowFeed=true uses CONVEYOR_FAR_SPEED (0.4) — same as PACED mode in TeleOp.
 * Omit slowFeed (or pass false) for full close-range conveyor speed.
 */
public class MoveAndShoot extends SequentialCommandGroup {

    /** Straight line, heading mode. */
    public MoveAndShoot(Follower follower, Pose targetPose,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, DriveToPose.HeadingMode headingMode) {
        this(follower, targetPose, feedTimeMs, overrideVelocity, isBlue, headingMode, false);
    }

    /** Straight line, heading mode, optional slow feed. */
    public MoveAndShoot(Follower follower, Pose targetPose,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, DriveToPose.HeadingMode headingMode, boolean slowFeed) {
        addCommands(
                new WindUpAndDrive(follower, targetPose, overrideVelocity, headingMode, 1.0),
                new Shoot(follower, feedTimeMs, overrideVelocity, isBlue, false, slowFeed)
        );
    }

    /** Straight line, piecewise heading. */
    public MoveAndShoot(Follower follower, Pose targetPose,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, PiecewiseHeading piecewiseHeading) {
        this(follower, targetPose, feedTimeMs, overrideVelocity, isBlue, piecewiseHeading, false);
    }

    /** Straight line, piecewise heading, optional slow feed. */
    public MoveAndShoot(Follower follower, Pose targetPose,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, PiecewiseHeading piecewiseHeading, boolean slowFeed) {
        addCommands(
                new WindUpAndDrive(follower, targetPose, overrideVelocity, piecewiseHeading, 1.0),
                new Shoot(follower, feedTimeMs, overrideVelocity, isBlue, false, slowFeed)
        );
    }

    /** Bezier curve, heading mode. */
    public MoveAndShoot(Follower follower, Pose[] waypoints,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, DriveToPose.HeadingMode headingMode) {
        this(follower, waypoints, feedTimeMs, overrideVelocity, isBlue, headingMode, false);
    }

    /** Bezier curve, heading mode, optional slow feed. */
    public MoveAndShoot(Follower follower, Pose[] waypoints,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, DriveToPose.HeadingMode headingMode, boolean slowFeed) {
        addCommands(
                new WindUpAndDrive(follower, waypoints, overrideVelocity, headingMode, 1.0),
                new Shoot(follower, feedTimeMs, overrideVelocity, isBlue, false, slowFeed)
        );
    }

    /** Bezier curve, piecewise heading. */
    public MoveAndShoot(Follower follower, Pose[] waypoints,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, PiecewiseHeading piecewiseHeading) {
        this(follower, waypoints, feedTimeMs, overrideVelocity, isBlue, piecewiseHeading, false);
    }

    /** Bezier curve, piecewise heading, optional slow feed. */
    public MoveAndShoot(Follower follower, Pose[] waypoints,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, PiecewiseHeading piecewiseHeading, boolean slowFeed) {
        addCommands(
                new WindUpAndDrive(follower, waypoints, overrideVelocity, piecewiseHeading, 1.0),
                new Shoot(follower, feedTimeMs, overrideVelocity, isBlue, false, slowFeed)
        );
    }
}