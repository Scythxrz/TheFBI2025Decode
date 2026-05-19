package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.globals.Constants;
import org.firstinspires.ftc.teamcode.config.globals.Poses;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * Shoot — spins up the flywheel and runs the conveyor for a fixed duration.
 *
 * State machine: SETTLING → SPINNING → FEEDING → DRAINING → DONE
 *
 * Tuning: set Constants.SHOOT_FEED_TIME_MS to how long the conveyor should run
 * after the flywheel reaches speed. Tune on the field to match your magazine size.
 *
 * Velocity modes:
 *   overrideVelocity > 0  — fixed ticks/s target
 *   overrideVelocity <= 0 — distance LUT from current pose
 *   velocityFF = true     — recessional FF for shoot-while-moving
 */
public class Shoot extends CommandBase {

    public enum State { SETTLING, SPINNING, FEEDING, DRAINING, DONE }
    private State state;

    private final Follower follower;
    private final long     feedTimeMs;
    private final double   overrideVelocity;
    private final boolean  isBlue;
    private final boolean  velocityFF;
    private final boolean  slowFeed;

    private static final long SPIN_UP_TIMEOUT_MS = 500;
    private static final long POST_SHOT_DRAIN_MS = 150;
    private static final long HEADING_SETTLE_MS  = 350;
    private static final long HEADING_TIMEOUT_MS = 350;

    private final Robot robot = Robot.getInstance();
    private long spinUpStart        = 0;
    private long feedStart          = 0;
    private long drainStart         = 0;
    private long headingSettleStart = 0;
    private long headingTimerStart  = 0;

    // ─── Constructors ─────────────────────────────────────────────────────────

    /** Fixed velocity, no velocity FF, full conveyor speed. */
    public Shoot(Follower follower, long feedTimeMs, double overrideVelocity, boolean isBlue) {
        this(follower, feedTimeMs, overrideVelocity, isBlue, false, false);
    }

    /** Fixed velocity, optional velocity FF, full conveyor speed. */
    public Shoot(Follower follower, long feedTimeMs, double overrideVelocity,
                 boolean isBlue, boolean velocityFF) {
        this(follower, feedTimeMs, overrideVelocity, isBlue, velocityFF, false);
    }

    /** Full constructor — slowFeed=true uses CONVEYOR_FAR_SPEED (0.4), like PACED mode in TeleOp. */
    public Shoot(Follower follower, long feedTimeMs, double overrideVelocity,
                 boolean isBlue, boolean velocityFF, boolean slowFeed) {
        this.follower         = follower;
        this.feedTimeMs       = feedTimeMs;
        this.overrideVelocity = overrideVelocity;
        this.isBlue           = isBlue;
        this.velocityFF       = velocityFF;
        this.slowFeed         = slowFeed;
        addRequirements(robot.flywheel, robot.conveyor);
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        spinUpStart        = 0;
        feedStart          = 0;
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
            case SETTLING:
                boolean headingOk       = headingError() < Constants.AIM_ANGLE_TOLERANCE;
                boolean headingTimedOut = System.currentTimeMillis() - headingTimerStart > HEADING_TIMEOUT_MS;

                if (headingOk) {
                    if (headingSettleStart == 0) headingSettleStart = System.currentTimeMillis();
                    if (System.currentTimeMillis() - headingSettleStart >= HEADING_SETTLE_MS) {
                        spinUpStart = System.currentTimeMillis();
                        state = State.SPINNING;
                    }
                } else {
                    headingSettleStart = 0;
                    if (headingTimedOut) {
                        spinUpStart = System.currentTimeMillis();
                        state = State.SPINNING;
                    }
                }
                break;

            case SPINNING:
                boolean ready    = robot.flywheel.atTarget();
                boolean timedOut = System.currentTimeMillis() - spinUpStart > SPIN_UP_TIMEOUT_MS;
                if (ready || timedOut) {
                    feedStart = System.currentTimeMillis();
                    robot.conveyor.feed(!slowFeed);
                    state = State.FEEDING;
                }
                break;

            case FEEDING:
                robot.conveyor.feed(!slowFeed);
                if (System.currentTimeMillis() - feedStart >= feedTimeMs) {
                    drainStart = System.currentTimeMillis();
                    state = State.DRAINING;
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
    public boolean isFinished() { return state == State.DONE; }

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