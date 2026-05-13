package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * ShootWhileMoving — fires for feedTimeMs while the robot is still driving.
 * Drive and Shoot run concurrently; isFinished() waits for both to complete.
 * Shoot uses velocityFF=true to compensate for the robot moving toward/away from the goal.
 */
public class ShootWhileMoving extends CommandBase {

    private final DriveToPose drive;
    private final Shoot       shoot;

    /** Straight line, heading mode. */
    public ShootWhileMoving(Follower follower, Pose targetPose,
                            long feedTimeMs, double overrideVelocity,
                            boolean isBlue, DriveToPose.HeadingMode headingMode) {
        this.drive = new DriveToPose(follower, targetPose, headingMode);
        this.shoot = new Shoot(follower, feedTimeMs, overrideVelocity, isBlue, true);
    }

    /** Straight line, piecewise heading. */
    public ShootWhileMoving(Follower follower, Pose targetPose,
                            long feedTimeMs, double overrideVelocity,
                            boolean isBlue, PiecewiseHeading piecewiseHeading) {
        this.drive = new DriveToPose(follower, new Pose[]{targetPose}, piecewiseHeading, 1.0);
        this.shoot = new Shoot(follower, feedTimeMs, overrideVelocity, isBlue, true);
    }

    /** Bezier curve, heading mode. */
    public ShootWhileMoving(Follower follower, Pose[] waypoints,
                            long feedTimeMs, double overrideVelocity,
                            boolean isBlue, DriveToPose.HeadingMode headingMode) {
        this.drive = new DriveToPose(follower, waypoints, headingMode);
        this.shoot = new Shoot(follower, feedTimeMs, overrideVelocity, isBlue, true);
    }

    /** Bezier curve, piecewise heading. */
    public ShootWhileMoving(Follower follower, Pose[] waypoints,
                            long feedTimeMs, double overrideVelocity,
                            boolean isBlue, PiecewiseHeading piecewiseHeading) {
        this.drive = new DriveToPose(follower, waypoints, piecewiseHeading, 1.0);
        this.shoot = new Shoot(follower, feedTimeMs, overrideVelocity, isBlue, true);
    }

    @Override public void initialize() { drive.initialize(); shoot.initialize(); }
    @Override public void execute()    { drive.execute();    shoot.execute();    }
    @Override public boolean isFinished() { return drive.isFinished() && shoot.isFinished(); }
    @Override public void end(boolean interrupted) { drive.end(interrupted); shoot.end(interrupted); }
}