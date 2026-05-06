package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.globals.Constants;
import org.firstinspires.ftc.teamcode.config.globals.Poses;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * Shoot — spins up the flywheel and fires a set number of balls.
 *
 * This is the shared shooting core used by MoveAndShoot and ShootWhileMoving.
 * It does NOT drive anywhere — pair it with DriveToPose via a command group.
 *
 * ── Firing modes ──────────────────────────────────────────────────────────────
 *   RAPID (default) — gate stays open between balls, counts detections continuously.
 *   PACED           — gate closes after each ball, waits for flywheel recovery.
 *
 * ── Velocity ──────────────────────────────────────────────────────────────────
 *   Pass overrideVelocity > 0 to use a fixed RPM.
 *   Pass overrideVelocity <= 0 to use the distance LUT from the current pose.
 *   Pass velocityFF = true to apply recessional velocity feed-forward (for shoot-while-moving).
 *
 * ── Timeout behaviour ─────────────────────────────────────────────────────────
 *   HEADING_SETTLE_MS  — heading must be within AIM_ANGLE_TOLERANCE for this long before firing.
 *   HEADING_TIMEOUT_MS — if heading never settles after this long, fire anyway.
 *   SPIN_UP_TIMEOUT_MS — if flywheel hasn't reached target after this long, fire anyway.
 *   NO_BALL_TIMEOUT_MS — if no ball is detected within this window after the last one
 *                        (or since feeding started), stop and move on. Handles under-loaded magazines.
 *   POST_SHOT_DRAIN_MS — after the last detected ball, keep conveyor running briefly
 *                        so the ball fully clears the shooter before stopping.
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *   // Fixed velocity, rapid fire:
 *   new Shoot(follower, 3, 1850, isBlue)
 *
 *   // Distance LUT, paced fire:
 *   new Shoot(follower, 3, -1, isBlue, FiringMode.PACED)
 *
 *   // With velocity feed-forward (shoot-while-moving):
 *   new Shoot(follower, 3, 1850, isBlue, FiringMode.RAPID, true)
 */
public class Shoot extends CommandBase {

    public enum FiringMode { RAPID, PACED }

    public enum State { SETTLING, SPINNING, FEEDING, RECOVERING, DRAINING, DONE }
    private State state;

    private Telemetry telemetry;
    private final Follower   follower;
    private final int        ballsToFire;
    private final double     overrideVelocity; // <= 0 means use distance LUT
    private final boolean    isBlue;
    private final FiringMode firingMode;
    private final boolean    velocityFF;       // true = recessional FF for shoot-while-moving

    private static final long SPIN_UP_TIMEOUT_MS   = 500;
    private static final long POST_SHOT_DRAIN_MS   = 150;
    private static final long NO_BALL_TIMEOUT_MS   = 800;
    private static final long HEADING_SETTLE_MS    = 350;  // how long heading must be stable before firing
    private static final long HEADING_TIMEOUT_MS   = 350;  // give up waiting for heading after this long

    private final Robot robot = Robot.getInstance();
    private int          shotsFired   = 0;
    private BallDetector detector;
    private long spinUpStart        = 0;
    private long drainStart         = 0;
    private long lastBallTime       = 0;
    private long headingSettleStart = 0; // when heading first came within tolerance
    private long headingTimerStart  = 0; // when SETTLING state began
    private JoinedTelemetry telemetryM;

    // ─── Constructors ─────────────────────────────────────────────────────────

    /** Fixed velocity, rapid fire, no velocity FF. */
    public Shoot(Follower follower, int ballsToFire, double overrideVelocity, boolean isBlue) {
        this(follower, ballsToFire, overrideVelocity, isBlue, FiringMode.RAPID, false);
    }

    /** Fixed velocity, chosen firing mode, no velocity FF. */
    public Shoot(Follower follower, int ballsToFire, double overrideVelocity,
                 boolean isBlue, FiringMode firingMode) {
        this(follower, ballsToFire, overrideVelocity, isBlue, firingMode, false);
    }

    /** Full constructor. */
    public Shoot(Follower follower, int ballsToFire, double overrideVelocity,
                 boolean isBlue, FiringMode firingMode, boolean velocityFF) {
        this.follower         = follower;
        this.ballsToFire      = ballsToFire;
        this.overrideVelocity = overrideVelocity;
        this.isBlue           = isBlue;
        this.firingMode       = firingMode;
        this.velocityFF       = velocityFF;
        addRequirements(robot.flywheel, robot.conveyor);
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        shotsFired   = 0;
        detector     = new BallDetector();
        spinUpStart  = System.currentTimeMillis();
        drainStart   = 0;
        lastBallTime = 0;
        state             = State.SETTLING;
        headingSettleStart = 0;
        headingTimerStart  = System.currentTimeMillis();
        telemetryM = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        updateFlywheel();
    }

    @Override
    public void execute() {
        updateFlywheel();
        switch (state) {
            case SETTLING:
                boolean headingOk = headingError() < Constants.AIM_ANGLE_TOLERANCE;
                boolean headingTimedOut = System.currentTimeMillis() - headingTimerStart > HEADING_TIMEOUT_MS;

                if (headingOk) {
                    if (headingSettleStart == 0) headingSettleStart = System.currentTimeMillis();
                    // Must stay within tolerance for HEADING_SETTLE_MS before proceeding
                    if (System.currentTimeMillis() - headingSettleStart >= HEADING_SETTLE_MS) {
                        spinUpStart = System.currentTimeMillis();
                        state = State.SPINNING;
                    }
                } else {
                    headingSettleStart = 0; // reset — heading drifted back out
                    if (headingTimedOut) {
                        // Heading never settled — fire anyway
                        spinUpStart = System.currentTimeMillis();
                        state = State.SPINNING;
                    }
                }
                break;

            case SPINNING:
                boolean ready    = robot.flywheel.atTarget();
                boolean timedOut = System.currentTimeMillis() - spinUpStart > SPIN_UP_TIMEOUT_MS;
                if (ready || timedOut) {
                    robot.conveyor.feed(true);
                    state = State.FEEDING;
                }
                break;

            case FEEDING:
                robot.conveyor.feed(true);
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
                    // RAPID: gate stays open, BallDetector cooldown handles spacing
                }
                break;

            case RECOVERING:
                boolean recovered   = robot.flywheel.atTarget();
                boolean recTimedOut = System.currentTimeMillis() - spinUpStart > SPIN_UP_TIMEOUT_MS;
                if (recovered || recTimedOut) {
                    lastBallTime = System.currentTimeMillis();
                    robot.conveyor.feed(true);
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
        return state == State.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        robot.conveyor.stop();
        if (interrupted) robot.flywheel.off();
    }

    // ─── Helpers ──────────────────────────────────────────────────────────────
    private void updateFlywheel() {
        if (overrideVelocity > 0) {
            robot.flywheel.setVelocity(overrideVelocity);
        } else if (velocityFF) {
            robot.flywheel.setVelocityForDistanceWithVelocityFF(
                    distanceToGoal(), recessionalVelocity());
        } else {
            robot.flywheel.setVelocityForDistance(distanceToGoal());
        }
    }

    /** Absolute heading error between current robot heading and the angle to the goal (radians). */
    private double headingError() {
        Pose goal = Poses.goal(isBlue);
        Pose pos  = follower.getPose();
        double targetHeading = Math.atan2(
                goal.getX() - pos.getX(),
                goal.getY() - pos.getY()
        ) + Math.PI;

        // Normalize both angles to (-π, π] before differencing
        // to avoid wrap-around issues near ±π boundary on Red
        double th = ((targetHeading + Math.PI) % (2 * Math.PI)) - Math.PI;
        double ph = ((pos.getHeading() + Math.PI) % (2 * Math.PI)) - Math.PI;
        double error = th - ph;
        while (error >  Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return Math.abs(error);
    }

    private double distanceToGoal() {
        Pose goal = Poses.goal(isBlue);
        double dx = goal.getX() - follower.getPose().getX();
        double dy = goal.getY() - follower.getPose().getY();
        return Math.hypot(dx, dy);
    }

    /** Robot velocity projected onto the away-from-goal axis (positive = moving away). */
    private double recessionalVelocity() {
        Pose goal = Poses.goal(isBlue);
        Pose pos  = follower.getPose();
        double dx   = pos.getX() - goal.getX();
        double dy   = pos.getY() - goal.getY();
        double dist = Math.hypot(dx, dy);
        if (dist < 1e-6) return 0;
        double vx = follower.getVelocity().getXComponent();
        double vy = follower.getVelocity().getYComponent();
        return (vx * dx + vy * dy) / dist;
    }
}