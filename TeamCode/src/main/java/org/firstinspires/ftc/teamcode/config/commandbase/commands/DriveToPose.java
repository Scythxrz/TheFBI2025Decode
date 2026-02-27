package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.HeadingInterpolator;
import com.seattlesolvers.solverslib.command.CommandBase;

/**
 * DriveToPose — drives to a target pose and finishes when Pedro reports !isBusy().
 *
 * Always pair with .withTimeout(ms) so a failed path can't freeze auto.
 *
 * Heading modes:
 *   LINEAR         — interpolates heading from start to end
 *   TANGENTIAL     — heading follows the curve direction
 *   TANGENTIAL_REV — tangential but reversed (robot drives backwards along the path)
 *
 * Usage examples:
 *
 *   // Straight line, linear heading, full speed
 *   new DriveToPose(follower, Poses.SCORE_CLOSE)
 *
 *   // Straight line, tangential heading
 *   new DriveToPose(follower, Poses.SCORE_CLOSE, HeadingMode.TANGENTIAL)
 *
 *   // Straight line, reversed tangential (drive backwards), 0.6 speed
 *   new DriveToPose(follower, Poses.SCORE_CLOSE, HeadingMode.TANGENTIAL_REV, 0.6)
 *
 *   // Bezier curve through waypoints, tangential heading
 *   new DriveToPose(follower, new Pose[]{Poses.PGP_MID, Poses.PGP_COLLECT}, HeadingMode.TANGENTIAL)
 */
public class DriveToPose extends CommandBase {

    public enum HeadingMode {
        LINEAR,         // setLinearHeadingInterpolation(start, end)
        TANGENTIAL,     // setTangentHeadingInterpolation()
        TANGENTIAL_REV  // setTangentHeadingInterpolation() + reverseHeadingInterpolation()
    }

    private final Follower    follower;
    private final Pose        targetPose;
    private final Pose[]      waypoints;   // null = straight line
    private final HeadingMode      headingMode;
    private final double           maxSpeed;
    private final PiecewiseHeading piecewiseHeading; // null = use headingMode

    // ─── Constructors — straight line ─────────────────────────────────────────

    /** Straight line, linear heading, full speed. */
    public DriveToPose(Follower follower, Pose target) {
        this(follower, target, HeadingMode.LINEAR, 1.0);
    }

    /** Straight line, chosen heading mode, full speed. */
    public DriveToPose(Follower follower, Pose target, HeadingMode headingMode) {
        this(follower, target, headingMode, 1.0);
    }

    /** Straight line, chosen heading mode, custom speed. */
    public DriveToPose(Follower follower, Pose target, HeadingMode headingMode, double maxSpeed) {
        this.follower         = follower;
        this.targetPose       = target;
        this.waypoints        = null;
        this.headingMode      = headingMode;
        this.maxSpeed         = maxSpeed;
        this.piecewiseHeading = null;
    }
    public DriveToPose(Follower follower, Pose target, HeadingMode headingMode, double maxSpeed, PiecewiseHeading piecewiseHeading) {
        this.follower         = follower;
        this.targetPose       = target;
        this.waypoints        = null;
        this.headingMode      = headingMode;
        this.maxSpeed         = maxSpeed;
        this.piecewiseHeading = piecewiseHeading;
    }

    /** Straight line, linear heading, custom speed (legacy shorthand). */
    public DriveToPose(Follower follower, Pose target, double maxSpeed) {
        this(follower, target, HeadingMode.LINEAR, maxSpeed);
    }

    // ─── Constructors — Bezier curve through waypoints ────────────────────────

    /** Bezier curve, chosen heading mode, full speed. */
    public DriveToPose(Follower follower, Pose[] waypoints, HeadingMode headingMode) {
        this(follower, waypoints, headingMode, 1.0);
    }

    /** Bezier curve, chosen heading mode, custom speed. */
    public DriveToPose(Follower follower, Pose[] waypoints, HeadingMode headingMode, double maxSpeed) {
        this.follower    = follower;
        this.targetPose  = waypoints[0]; // first element is the endpoint
        this.waypoints   = waypoints;
        this.headingMode = headingMode;
        this.maxSpeed    = maxSpeed;
        this.piecewiseHeading = null;

    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        Pose from = follower.getPose();
        Path pathObj;

        if (waypoints != null && waypoints.length >= 2) {
            // BezierCurve order: [start, control point(s)..., end]
            // Caller passes [control point(s)..., endpoint] so we reverse
            // the waypoints array: endpoint goes last, controls go in between.
            Pose[] allPoses = new Pose[waypoints.length + 1];
            allPoses[0] = from;
            // Reverse: last waypoint (endpoint) stays last, earlier ones are controls
            for (int i = 0; i < waypoints.length - 1; i++) {
                allPoses[i + 1] = waypoints[waypoints.length - 2 - i]; // control points reversed
            }
            allPoses[allPoses.length - 1] = waypoints[0]; // first waypoint = endpoint
            pathObj = new Path(new BezierCurve(allPoses));
        } else {
            // Straight line
            pathObj = new Path(new BezierLine(from, targetPose));
        }

        applyHeading(pathObj, from);

        PathChain chain = follower.pathBuilder()
                .addPath(pathObj)
                .setTimeoutConstraint(500)
                .build();

        follower.followPath(chain, maxSpeed, true);
    }

    @Override
    public void execute() { }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) follower.breakFollowing();
    }

    // ─── Helper ───────────────────────────────────────────────────────────────

    private void applyHeading(Path path, Pose from) {
        if (piecewiseHeading != null) {
            path.setHeadingInterpolation(piecewiseHeading.build());
            return;
        }
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
}