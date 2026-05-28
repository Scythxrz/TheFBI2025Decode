package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

/**
 * MoveAndShoot — drives to a pose then fires after arriving.
 * WindUpAndDrive spins the flywheel during transit so it is ready on arrival.
 *
 * Pass paced=true (shootF in Auton) to use paced mode — the conveyor pauses
 * after each detected ball and waits for flywheel recovery before the next.
 */
public class MoveAndShoot extends SequentialCommandGroup {

    /** Straight line, heading mode. */
    public MoveAndShoot(Follower follower, Pose targetPose,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, DriveToPose.HeadingMode headingMode) {
        addCommands(
                new WindUpAndDrive(follower, targetPose, overrideVelocity, headingMode, 1.0),
                new Shoot(follower, feedTimeMs, overrideVelocity, isBlue)
        );
    }

    /** Straight line, piecewise heading. */
    public MoveAndShoot(Follower follower, Pose targetPose,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, PiecewiseHeading piecewiseHeading) {
        addCommands(
                new WindUpAndDrive(follower, targetPose, overrideVelocity, piecewiseHeading, 1.0),
                new Shoot(follower, feedTimeMs, overrideVelocity, isBlue)
        );
    }

    /** Straight line, piecewise heading, paced mode flag. */
    public MoveAndShoot(Follower follower, Pose targetPose,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, PiecewiseHeading piecewiseHeading, boolean paced) {
        addCommands(
                new WindUpAndDrive(follower, targetPose, overrideVelocity, piecewiseHeading, 1.0),
                new Shoot(follower, feedTimeMs, overrideVelocity, isBlue, false, true, paced)
        );
    }

    /** Bezier curve, heading mode. */
    public MoveAndShoot(Follower follower, Pose[] waypoints,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, DriveToPose.HeadingMode headingMode) {
        addCommands(
                new WindUpAndDrive(follower, waypoints, overrideVelocity, headingMode, 1.0),
                new Shoot(follower, feedTimeMs, overrideVelocity, isBlue)
        );
    }

    /** Bezier curve, piecewise heading. */
    public MoveAndShoot(Follower follower, Pose[] waypoints,
                        long feedTimeMs, double overrideVelocity,
                        boolean isBlue, PiecewiseHeading piecewiseHeading) {
        addCommands(
                new WindUpAndDrive(follower, waypoints, overrideVelocity, piecewiseHeading, 1.0),
                new Shoot(follower, feedTimeMs, overrideVelocity, isBlue)
        );
    }
}