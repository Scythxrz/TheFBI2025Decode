package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.globals.Poses;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * MoveAndShoot — drives the path while winding up, then fires after path is done.
 *
 * Unlike ShootWhileMoving, feeding does NOT begin until the path finishes.
 * Use this when you need the robot at the exact target pose before firing
 * (e.g. far-zone shots where heading precision matters).
 *
 * Same FiringMode and global feeding timeout as ShootWhileMoving:
 *
 *   RAPID (default) — gate stays open between balls, counts detections continuously.
 *   PACED           — gate closes after each ball, waits for flywheel recovery.
 *
 *   FEEDING_TIMEOUT_MS — single global timer that starts when feeding first begins.
 *     If it expires before all balls are counted (e.g. only 2 loaded when 3 expected),
 *     the command finishes and auto continues. One timer for the whole sequence.
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *   // Rapid fire after arriving (default):
 *   new MoveAndShoot(follower, Poses.SCORE_CLOSE, 3, 1850, isBlue)
 *
 *   // Paced fire after arriving (far zone):
 *   new MoveAndShoot(follower, Poses.FAR_SCORE, 3, 2450, isBlue, FiringMode.PACED)
 */
public class MoveAndShoot extends CommandBase {

    public enum FiringMode { RAPID, PACED }

    private enum State { DRIVING, SPINNING, FEEDING, RECOVERING, DONE }
    private State state;

    private final Follower   follower;
    private final Pose       targetPose;
    private final int        ballsToFire;
    private final double     overrideVelocity;
    private final boolean    isBlue;
    private final FiringMode firingMode;

    private static final long SPIN_UP_TIMEOUT_MS = 2000;
    private static final long FEEDING_TIMEOUT_MS = 3000;

    private final Robot robot = Robot.getInstance();
    private int  shotsFired   = 0;
    private long spinUpStart  = 0;
    private long feedingStart = 0;

    // ─── Constructors ─────────────────────────────────────────────────────────

    /** Rapid fire, distance LUT velocity. */
    public MoveAndShoot(Follower follower, Pose targetPose, int ballsToFire, boolean isBlue) {
        this(follower, targetPose, ballsToFire, -1, isBlue, FiringMode.RAPID);
    }

    /** Rapid fire, explicit velocity. */
    public MoveAndShoot(Follower follower, Pose targetPose, int ballsToFire,
                        double overrideVelocity, boolean isBlue) {
        this(follower, targetPose, ballsToFire, overrideVelocity, isBlue, FiringMode.RAPID);
    }

    /** Explicit velocity + firing mode. */
    public MoveAndShoot(Follower follower, Pose targetPose, int ballsToFire,
                        double overrideVelocity, boolean isBlue, FiringMode firingMode) {
        this.follower         = follower;
        this.targetPose       = targetPose;
        this.ballsToFire      = ballsToFire;
        this.overrideVelocity = overrideVelocity;
        this.isBlue           = isBlue;
        this.firingMode       = firingMode;
        addRequirements(robot.flywheel, robot.conveyor);
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        shotsFired   = 0;
        spinUpStart  = System.currentTimeMillis();
        feedingStart = 0;
        state        = State.DRIVING;

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

        // Global feeding timeout
        if (feedingStart > 0 && System.currentTimeMillis() - feedingStart > FEEDING_TIMEOUT_MS) {
            robot.conveyor.stop();
            robot.flywheel.off();
            state = State.DONE;
            return;
        }

        switch (state) {

            case DRIVING:
                if (!follower.isBusy()) {
                    spinUpStart = System.currentTimeMillis();
                    state = State.SPINNING;
                }
                break;

            case SPINNING:
                boolean ready    = robot.flywheel.atTarget();
                boolean timedOut = System.currentTimeMillis() - spinUpStart > SPIN_UP_TIMEOUT_MS;

                if (ready || timedOut) {
                    feedingStart = System.currentTimeMillis(); // global timer starts NOW
                    robot.conveyor.feed();
                    state = State.FEEDING;
                }
                break;

            case FEEDING:
                if (robot.flywheel.ballDetected()) {
                    shotsFired++;

                    if (shotsFired >= ballsToFire) {
                        robot.conveyor.stop();
                        robot.flywheel.off();
                        state = State.DONE;
                    } else if (firingMode == FiringMode.PACED) {
                        robot.conveyor.stop();
                        spinUpStart = System.currentTimeMillis();
                        state = State.RECOVERING;
                    }
                    // RAPID: gate stays open, global timer handles stuck-ball case
                }
                break;

            case RECOVERING:
                boolean recovered   = robot.flywheel.atTarget();
                boolean recTimedOut = System.currentTimeMillis() - spinUpStart > SPIN_UP_TIMEOUT_MS;

                if (recovered || recTimedOut) {
                    robot.conveyor.feed();
                    state = State.FEEDING;
                }
                break;

            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        robot.conveyor.stop();
        if (interrupted) robot.flywheel.off();
    }

    // ─── Helpers ──────────────────────────────────────────────────────────────

    private void spinUpFlywheel() {
        if (overrideVelocity > 0) {
            robot.flywheel.setVelocity(overrideVelocity);
        } else {
            robot.flywheel.setVelocityForDistance(distanceToGoal());
        }
    }

    private double distanceToGoal() {
        Pose goal = Poses.goal(isBlue);
        double dx = goal.getX() - follower.getPose().getX();
        double dy = goal.getY() - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }
}