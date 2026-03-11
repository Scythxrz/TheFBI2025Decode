package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.globals.Poses;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * ShootWhileMoving — fires balls while the robot is still driving the path.
 *
 * Flywheel spins up immediately. As soon as it hits target velocity (or spin-up
 * times out), the conveyor opens and balls are fired mid-path.
 *
 * ── Firing modes ──────────────────────────────────────────────────────────────
 *
 *   RAPID (default)
 *     Gate opens once and stays open. Ball detections are counted continuously
 *     with no pause between shots. Use for close-range where the flywheel
 *     recovers fast enough without needing a break.
 *
 *   PACED
 *     Gate closes after each detected ball, waits for flywheel to recover back
 *     to target speed, then reopens. Use for far shots where velocity sag hurts.
 *
 * ── Global feeding timeout ────────────────────────────────────────────────────
 *   A single timer starts the moment feeding begins. If the timer expires before
 *   all balls are counted (e.g. only 2 loaded when 3 expected), the command
 *   finishes immediately — auto never stalls. The timer covers the ENTIRE feeding
 *   sequence, not just each individual ball.
 *
 *   POST_SHOT_DRAIN_MS — how long to keep the conveyor running after the last
 *     detected ball, ensuring it fully clears the shooter before stopping.
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *   // Rapid fire, distance LUT:
 *   new ShootWhileMoving(follower, Poses.SCORE_CLOSE, 3, isBlue)
 *
 *   // Rapid fire, explicit velocity:
 *   new ShootWhileMoving(follower, Poses.SCORE_CLOSE, 3, 1850, isBlue)
 *
 *   // Paced fire (far zone):
 *   new ShootWhileMoving(follower, Poses.FAR_SCORE, 3, 2450, isBlue, FiringMode.PACED)
 */
public class ShootWhileMoving extends CommandBase {

    public enum FiringMode { RAPID, PACED }

    public enum HeadingMode { LINEAR, TANGENTIAL, TANGENTIAL_REV }

    private enum State { SPINNING, FEEDING, RECOVERING, DRAINING, DONE }
    private State state;

    private final Follower   follower;
    private final Pose       targetPose;
    private final int        ballsToFire;
    private final double     overrideVelocity;
    private final boolean    isBlue;
    private final FiringMode      firingMode;
    private final HeadingMode     headingMode;
    private final PiecewiseHeading piecewiseHeading; // null = use headingMode

    private static final long SPIN_UP_TIMEOUT_MS = 2000;
    private static final long POST_SHOT_DRAIN_MS  = 150;  // conveyor drain after last ball
    private static final long NO_BALL_TIMEOUT_MS  = 800;  // if no ball detected within this window, stop and move on

    private final Robot robot = Robot.getInstance();
    private int          shotsFired = 0;
    private BallDetector detector;
    private long spinUpStart  = 0;
    private long drainStart   = 0;
    private long lastBallTime = 0; // time of last ball detection; 0 = none yet

    // ─── Constructors ─────────────────────────────────────────────────────────

    /** Rapid fire, distance LUT velocity. */
    public ShootWhileMoving(Follower follower, Pose targetPose, int ballsToFire, boolean isBlue) {
        this(follower, targetPose, ballsToFire, -1, isBlue, FiringMode.RAPID);
    }

    /** Rapid fire, explicit velocity. */
    public ShootWhileMoving(Follower follower, Pose targetPose, int ballsToFire,
                            double overrideVelocity, boolean isBlue) {
        this(follower, targetPose, ballsToFire, overrideVelocity, isBlue, FiringMode.RAPID);
    }

    /** Explicit velocity + firing mode, linear heading (default). */
    public ShootWhileMoving(Follower follower, Pose targetPose, int ballsToFire,
                            double overrideVelocity, boolean isBlue, FiringMode firingMode) {
        this(follower, targetPose, ballsToFire, overrideVelocity, isBlue, firingMode, HeadingMode.LINEAR);
    }

    /** Full constructor — explicit velocity, firing mode, and heading mode. */
    public ShootWhileMoving(Follower follower, Pose targetPose, int ballsToFire,
                            double overrideVelocity, boolean isBlue, FiringMode firingMode, HeadingMode headingMode) {
        this.follower         = follower;
        this.targetPose       = targetPose;
        this.ballsToFire      = ballsToFire;
        this.overrideVelocity = overrideVelocity;
        this.isBlue           = isBlue;
        this.firingMode       = firingMode;
        this.headingMode      = headingMode;
        this.piecewiseHeading = null;
        addRequirements(robot.flywheel, robot.conveyor);
    }

    /** Full constructor — explicit velocity, firing mode, and piecewise heading. */
    public ShootWhileMoving(Follower follower, Pose targetPose, int ballsToFire,
                            double overrideVelocity, boolean isBlue, FiringMode firingMode, PiecewiseHeading piecewiseHeading) {
        this.follower         = follower;
        this.targetPose       = targetPose;
        this.ballsToFire      = ballsToFire;
        this.overrideVelocity = overrideVelocity;
        this.isBlue           = isBlue;
        this.firingMode       = firingMode;
        this.headingMode      = null;
        this.piecewiseHeading = piecewiseHeading;
        addRequirements(robot.flywheel, robot.conveyor);
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        shotsFired  = 0;
        detector    = new BallDetector();
        spinUpStart = System.currentTimeMillis();
        drainStart  = 0; // not started yet — set when we first enter FEEDING
        state        = State.SPINNING;

        spinUpFlywheel();

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                .setTimeoutConstraint(300)
                .build();

        follower.followPath(path, 1.0, true);
    }

    @Override
    public void execute() {
        spinUpFlywheel();

        switch (state) {

            case SPINNING:
                boolean ready    = robot.flywheel.atTarget();
                boolean timedOut = System.currentTimeMillis() - spinUpStart > SPIN_UP_TIMEOUT_MS;

                if (ready || timedOut) {
                    robot.conveyor.feed();
                    state = State.FEEDING;
                }
                break;

            case FEEDING:
                if (lastBallTime == 0) lastBallTime = System.currentTimeMillis();

                if (System.currentTimeMillis() - lastBallTime > NO_BALL_TIMEOUT_MS) {
                    drainStart = System.currentTimeMillis();
                    state = State.DRAINING;
                    break;
                }

                if (detector.update()) {
                    shotsFired++;
                    lastBallTime = System.currentTimeMillis();
                    if (shotsFired >= ballsToFire) {
                        drainStart = System.currentTimeMillis();
                        state = State.DRAINING;
                    } else if (firingMode == FiringMode.PACED) {
                        robot.conveyor.stop();
                        spinUpStart = System.currentTimeMillis();
                        state = State.RECOVERING;
                    }
                }
                break;

            case RECOVERING:
                boolean recovered   = robot.flywheel.atTarget();
                boolean recTimedOut = System.currentTimeMillis() - spinUpStart > SPIN_UP_TIMEOUT_MS;

                if (recovered || recTimedOut) {
                    lastBallTime = System.currentTimeMillis(); // reset clock on re-entry
                    robot.conveyor.feed();
                    state = State.FEEDING;
                }
                break;

            case DRAINING:
                if (System.currentTimeMillis() - drainStart >= POST_SHOT_DRAIN_MS) {
                    robot.conveyor.stop();
                    robot.flywheel.off();
                    state = State.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE && !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        robot.conveyor.stop();
        if (interrupted) robot.flywheel.off();
    }

    // ─── Helpers ──────────────────────────────────────────────────────────────

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

    private void spinUpFlywheel() {
        if (overrideVelocity > 0) {
            robot.flywheel.setVelocity(overrideVelocity);
        } else {
            robot.flywheel.setVelocityForDistanceWithVelocityFF(
                    distanceToGoal(), recessionalVelocity());
        }
    }

    private double distanceToGoal() {
        Pose goal = Poses.goal(isBlue);
        double dx = goal.getX() - follower.getPose().getX();
        double dy = goal.getY() - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }

    /** Robot velocity projected onto the goal-away axis (positive = moving away from goal). */
    private double recessionalVelocity() {
        Pose goal = Poses.goal(isBlue);
        Pose pos  = follower.getPose();
        double dx   = pos.getX() - goal.getX(); // away-from-goal vector
        double dy   = pos.getY() - goal.getY();
        double dist = Math.hypot(dx, dy);
        if (dist < 1e-6) return 0; // at the goal, avoid divide-by-zero
        // Dot robot velocity onto the unit away-vector
        double vx = follower.getVelocity().getXComponent();
        double vy = follower.getVelocity().getYComponent();
        return (vx * dx + vy * dy) / dist;
    }
}