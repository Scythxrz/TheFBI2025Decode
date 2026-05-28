package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.globals.Constants;
import org.firstinspires.ftc.teamcode.config.globals.Poses;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * Shoot — spins up the flywheel and feeds balls into it.
 *
 * Two feeding modes, selected by the {@code paced} constructor flag:
 *
 *   TIMED (paced=false, default):
 *     Runs the conveyor continuously for feedTimeMs then stops.
 *     State machine: SETTLING → SPINNING → FEEDING → DRAINING → DONE
 *     Use for close-zone shots where the flywheel recovers quickly.
 *
 *   PACED (paced=true):
 *     After each ball is detected (velocity drop in Flywheel.ballDetected()),
 *     the conveyor pauses and waits for the flywheel to return to target speed
 *     before feeding the next ball. feedTimeMs is a hard total-time safety cap
 *     that prevents the command from hanging if ball detection fails.
 *     State machine: SETTLING → SPINNING → FEEDING ↔ RECOVERING → DRAINING → DONE
 *     Use for far-zone shots where the flywheel needs time to recover.
 *
 * Velocity modes:
 *   overrideVelocity > 0  — fixed ticks/s target
 *   overrideVelocity <= 0 — distance LUT from current pose
 *   velocityFF = true     — recessional FF for shoot-while-moving
 */
public class Shoot extends CommandBase {

    public enum State { SETTLING, SPINNING, FEEDING, RECOVERING, DRAINING, DONE }
    private State state;

    private final Follower follower;
    private final long     feedTimeMs;       // total time cap (also the feed duration in timed mode)
    private final double   overrideVelocity;
    private final boolean  isBlue;
    private final boolean  velocityFF;
    private final boolean  slowFeed;
    private final boolean  paced;            // true = wait for flywheel recovery between each ball

    private static final long SPIN_UP_TIMEOUT_MS = 500;
    private static final long POST_SHOT_DRAIN_MS = 150;
    private static final long HEADING_SETTLE_MS  = 350;
    private static final long HEADING_TIMEOUT_MS = 350;

    private final Robot robot = Robot.getInstance();
    private long spinUpStart        = 0;
    private long feedStart          = 0;     // set once when FEEDING begins; used as total-time cap
    private long recoverStart       = 0;     // set each time RECOVERING begins
    private long drainStart         = 0;
    private long headingSettleStart = 0;
    private long headingTimerStart  = 0;

    // ─── Constructors ─────────────────────────────────────────────────────────

    /** Timed mode, full conveyor speed. */
    public Shoot(Follower follower, long feedTimeMs, double overrideVelocity, boolean isBlue) {
        this(follower, feedTimeMs, overrideVelocity, isBlue, false, false, false);
    }

    /** Timed mode, optional velocity FF. */
    public Shoot(Follower follower, long feedTimeMs, double overrideVelocity,
                 boolean isBlue, boolean velocityFF) {
        this(follower, feedTimeMs, overrideVelocity, isBlue, velocityFF, false, false);
    }

    /** Full constructor. slowFeed uses CONVEYOR_FAR_SPEED; paced waits for flywheel recovery. */
    public Shoot(Follower follower, long feedTimeMs, double overrideVelocity,
                 boolean isBlue, boolean velocityFF, boolean slowFeed, boolean paced) {
        this.follower         = follower;
        this.feedTimeMs       = feedTimeMs;
        this.overrideVelocity = overrideVelocity;
        this.isBlue           = isBlue;
        this.velocityFF       = velocityFF;
        this.slowFeed         = slowFeed;
        this.paced            = paced;
        addRequirements(robot.flywheel, robot.conveyor);
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        spinUpStart        = 0;
        feedStart          = 0;
        recoverStart       = 0;
        drainStart         = 0;
        headingSettleStart = 0;
        headingTimerStart  = System.currentTimeMillis();
        state              = State.SETTLING;
        updateFlywheel();
    }

    @Override
    public void execute() {
        updateFlywheel();

        switch (state) {
            case SETTLING: {
                boolean headingOk      = headingError() < Constants.AIM_ANGLE_TOLERANCE;
                boolean headingExpired = elapsed(headingTimerStart) > HEADING_TIMEOUT_MS;
                if (headingOk) {
                    if (headingSettleStart == 0) headingSettleStart = now();
                    if (elapsed(headingSettleStart) >= HEADING_SETTLE_MS) {
                        spinUpStart = now();
                        state = State.SPINNING;
                    }
                } else {
                    headingSettleStart = 0;
                    if (headingExpired) { spinUpStart = now(); state = State.SPINNING; }
                }
                break;
            }
            case SPINNING: {
                boolean ready   = robot.flywheel.atTarget();
                boolean expired = elapsed(spinUpStart) > SPIN_UP_TIMEOUT_MS;
                if (ready || expired) {
                    feedStart = now();
                    robot.conveyor.feed(!slowFeed);
                    state = State.FEEDING;
                }
                break;
            }
            case FEEDING: {
                // Hard total-time cap — prevents the command hanging if detection fails
                if (elapsed(feedStart) >= feedTimeMs) {
                    drainStart = now();
                    state = State.DRAINING;
                    break;
                }
                robot.conveyor.feed(!slowFeed);
                // In paced mode, detect each ball and pause for flywheel recovery
                if (paced && robot.flywheel.ballDetected()) {
                    robot.conveyor.stop();
                    recoverStart = now();
                    state = State.RECOVERING;
                }
                break;
            }
            case RECOVERING: {
                // Hard total-time cap still applies
                if (elapsed(feedStart) >= feedTimeMs) {
                    drainStart = now();
                    state = State.DRAINING;
                    break;
                }
                boolean recovered = robot.flywheel.atTarget();
                boolean timedOut  = elapsed(recoverStart) > Constants.PACED_RECOVERY_TIMEOUT_MS;
                if (recovered || timedOut) {
                    robot.conveyor.feed(!slowFeed);
                    state = State.FEEDING;
                }
                break;
            }
            case DRAINING: {
                if (elapsed(drainStart) >= POST_SHOT_DRAIN_MS) {
                    robot.conveyor.stop();
                    robot.flywheel.off();
                    state = State.DONE;
                }
                break;
            }
            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() { return state == State.DONE; }

    @Override
    public void end(boolean interrupted) {
        robot.conveyor.stop();
        if (interrupted) robot.flywheel.off();
    }

    // ─── Helpers ──────────────────────────────────────────────────────────────

    private long now() { return System.currentTimeMillis(); }
    private long elapsed(long start) { return now() - start; }

    private void updateFlywheel() {
        if (overrideVelocity > 0) {
            robot.flywheel.setVelocity(overrideVelocity);
        } else if (velocityFF) {
            robot.flywheel.setVelocityForDistanceWithVelocityFF(distanceToGoal(), recessionalVelocity());
        } else {
            robot.flywheel.setVelocityForDistance(distanceToGoal());
        }
    }

    private double headingError() {
        Pose goal = Poses.goal(isBlue);
        Pose pos  = follower.getPose();
        double targetHeading = Math.atan2(goal.getX() - pos.getX(), goal.getY() - pos.getY()) + Math.PI;
        double th    = ((targetHeading + Math.PI) % (2 * Math.PI)) - Math.PI;
        double ph    = ((pos.getHeading() + Math.PI) % (2 * Math.PI)) - Math.PI;
        double error = th - ph;
        while (error >  Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return Math.abs(error);
    }

    private double distanceToGoal() {
        Pose goal = Poses.goal(isBlue);
        return Math.hypot(goal.getX() - follower.getPose().getX(), goal.getY() - follower.getPose().getY());
    }

    private double recessionalVelocity() {
        Pose goal = Poses.goal(isBlue);
        Pose pos  = follower.getPose();
        double dx = pos.getX() - goal.getX(), dy = pos.getY() - goal.getY();
        double dist = Math.hypot(dx, dy);
        if (dist < 1e-6) return 0;
        return (follower.getVelocity().getXComponent() * dx + follower.getVelocity().getYComponent() * dy) / dist;
    }
}