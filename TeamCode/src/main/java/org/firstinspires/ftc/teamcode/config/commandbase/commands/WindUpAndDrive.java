package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.globals.Constants;
import org.firstinspires.ftc.teamcode.config.globals.Poses;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * WindUpAndDrive — drives a path while spinning the flywheel up to speed.
 * The conveyor stays closed the entire time — no feeding happens here.
 *
 * Heading modes (same as DriveToPose):
 *   LINEAR         — interpolates heading from start to end
 *   TANGENTIAL     — heading follows the curve direction
 *   TANGENTIAL_REV — tangential but reversed (robot drives backwards)
 *
 * Usage examples:
 *
 *   // Straight line, linear heading, explicit velocity
 *   new WindUpAndDrive(follower, Poses.PGP_COLLECT, 1850)
 *
 *   // Tangential heading, slow collect speed
 *   new WindUpAndDrive(follower, Poses.PGP_COLLECT, 1850, HeadingMode.TANGENTIAL, 0.6)
 *
 *   // Drive backwards (reversed tangential), velocity from distance LUT
 *   new WindUpAndDrive(follower, Poses.SCORE_CLOSE, Poses.SCORE_CLOSE, isBlue,
 *                      HeadingMode.TANGENTIAL_REV, 1.0)
 */
public class WindUpAndDrive extends CommandBase {

    // Re-export so callers only need to import one class
    public enum HeadingMode {
        LINEAR,
        TANGENTIAL,
        TANGENTIAL_REV
    }

    private final Follower    follower;
    private final Pose        targetPose;
    private final Pose[]      waypoints;
    private final double      flywheelVelocity;
    private final double      driveSpeed;
    private final HeadingMode      headingMode;
    private final PiecewiseHeading piecewiseHeading; // null = use headingMode

    private final Robot robot = Robot.getInstance();

    // ─── Constructors — explicit velocity ────────────────────────────────────

    /** Straight line, linear heading, full speed. */
    public WindUpAndDrive(Follower follower, Pose targetPose, double flywheelVelocity) {
        this(follower, targetPose, flywheelVelocity, HeadingMode.LINEAR, 1.0);
    }

    /** Straight line, linear heading, custom speed. */
    public WindUpAndDrive(Follower follower, Pose targetPose, double flywheelVelocity, double driveSpeed) {
        this(follower, targetPose, flywheelVelocity, HeadingMode.LINEAR, driveSpeed);
    }

    /** Straight line, chosen heading mode, custom speed. */
    public WindUpAndDrive(Follower follower, Pose targetPose, double flywheelVelocity,
                          HeadingMode headingMode, double driveSpeed) {
        this.follower         = follower;
        this.targetPose       = targetPose;
        this.waypoints        = null;
        this.flywheelVelocity = flywheelVelocity;
        this.headingMode      = headingMode;
        this.driveSpeed       = driveSpeed;
        this.piecewiseHeading = null;
    }

    /** Straight line, piecewise heading, custom speed, explicit velocity. */
    public WindUpAndDrive(Follower follower, Pose targetPose, double flywheelVelocity,
                          PiecewiseHeading piecewiseHeading, double driveSpeed) {
        this.follower         = follower;
        this.targetPose       = targetPose;
        this.waypoints        = null;
        this.flywheelVelocity = flywheelVelocity;
        this.headingMode      = null;
        this.driveSpeed       = driveSpeed;
        this.piecewiseHeading = piecewiseHeading;
    }

    // ─── Constructors — distance LUT velocity ────────────────────────────────

    /** Straight line, linear heading, full speed, velocity from LUT. */
    public WindUpAndDrive(Follower follower, Pose targetPose, Pose shootFromPose, boolean isBlue) {
        this(follower, targetPose, computeLUTVelocity(shootFromPose, isBlue), HeadingMode.LINEAR, 1.0);
    }

    /** Straight line, chosen heading mode, custom speed, velocity from LUT. */
    public WindUpAndDrive(Follower follower, Pose targetPose, Pose shootFromPose, boolean isBlue,
                          HeadingMode headingMode, double driveSpeed) {
        this(follower, targetPose, computeLUTVelocity(shootFromPose, isBlue), headingMode, driveSpeed);
    }

    // ─── Constructors — Bezier curve through waypoints ───────────────────────

    /** Bezier curve, chosen heading mode, custom speed, explicit velocity. */
    public WindUpAndDrive(Follower follower, Pose[] waypoints, double flywheelVelocity,
                          HeadingMode headingMode, double driveSpeed) {
        this.follower         = follower;
        this.targetPose       = waypoints[waypoints.length - 1];
        this.waypoints        = waypoints;
        this.flywheelVelocity = flywheelVelocity;
        this.headingMode      = headingMode;
        this.driveSpeed       = driveSpeed;
        this.piecewiseHeading = null;
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        robot.flywheel.setVelocity(flywheelVelocity);

        Pose from = follower.getPose();
        Path pathObj;

        if (waypoints != null && waypoints.length >= 2) {
            Pose[] allPoses = new Pose[waypoints.length + 1];
            allPoses[0] = from;
            System.arraycopy(waypoints, 0, allPoses, 1, waypoints.length);
            pathObj = new Path(new BezierCurve(allPoses));
        } else {
            pathObj = new Path(new BezierLine(from, targetPose));
        }

        applyHeading(pathObj, from);

        PathChain chain = follower.pathBuilder()
                .addPath(pathObj)
                .setTimeoutConstraint(300)
                .build();

        follower.followPath(chain, driveSpeed, true);
    }

    @Override
    public void execute() {
        // Keep velocity fresh — don't let it drift between ticks
        robot.flywheel.setVelocity(flywheelVelocity);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) follower.breakFollowing();
        // Flywheel stays running — caller decides when to stop it
    }

    // ─── Helpers ─────────────────────────────────────────────────────────────

    private void applyHeading(Path path, Pose from) {
        if (piecewiseHeading != null) {
            path.setHeadingInterpolation(piecewiseHeading.build());
            return;
        }
        if (headingMode == null) return;
        switch (headingMode) {
            case TANGENTIAL:
                path.setTangentHeadingInterpolation();
                break;
            case TANGENTIAL_REV:
                path.setTangentHeadingInterpolation();
                path.reverseHeadingInterpolation();
                break;
            case LINEAR:
            default:
                path.setLinearHeadingInterpolation(from.getHeading(), targetPose.getHeading());
                break;
        }
    }

    private static double computeLUTVelocity(Pose shootFromPose, boolean isBlue) {
        Pose goal = Poses.goal(isBlue);
        double dist = Math.hypot(
                goal.getX() - shootFromPose.getX(),
                goal.getY() - shootFromPose.getY()
        );
        double[][] lut = Constants.SHOOTER_LUT;
        if (dist <= lut[0][0])              return lut[0][1];
        if (dist >= lut[lut.length - 1][0]) return lut[lut.length - 1][1];
        for (int i = 0; i < lut.length - 1; i++) {
            if (dist >= lut[i][0] && dist <= lut[i + 1][0]) {
                double d1 = lut[i][0], p1 = lut[i][1];
                double d2 = lut[i + 1][0], p2 = lut[i + 1][1];
                return p1 + (dist - d1) * (p2 - p1) / (d2 - d1);
            }
        }
        return lut[lut.length - 1][1];
    }
}