package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.globals.Poses;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * MoveAndShoot — follows path, then fires ballsToFire balls.
 *
 * The flywheel spins up immediately, and as soon as it hits target velocity
 * the conveyor opens once path is finished.
 *
 * State machine:
 *   DRIVING_AND_SPINNING  — path running, flywheel warming up, waiting for atTarget()
 *   FEEDING               — flywheel ready, conveyor open, counting ball detections
 *   DONE                  — all balls fired (path may still be finishing — that's fine)
 *
 * The command finishes when all balls are fired AND the path is done.
 *
 * Usage:
 *   new MoveAndShoot(follower, Poses.SCORE_CLOSE, 1, isBlue).withTimeout(3000)
 *
 *   // Explicit velocity override (skip distance LUT):
 *   new MoveAndShoot(follower, Poses.SCORE_CLOSE, 3, 1850, isBlue).withTimeout(4000)
 */
public class MoveAndShoot extends CommandBase {

    private enum State { DRIVING_AND_SPINNING, FEEDING, DONE }
    private State state;

    private final Follower follower;
    private final Pose     targetPose;
    private final int      ballsToFire;
    private final double   overrideVelocity; // -1 = use distance LUT
    private final boolean  isBlue;

    // How long to wait for flywheel to reach speed before firing anyway (ms)
    // Set high enough that a sag-y battery doesn't cause early misfires
    private static final long SPIN_UP_TIMEOUT_MS = 2000;

    private final Robot robot = Robot.getInstance();
    private int  shotsFired  = 0;
    private long spinUpStart = 0;

    // ─── Constructors ─────────────────────────────────────────────────────────

    /** Distance LUT velocity. */
    public MoveAndShoot(Follower follower, Pose targetPose, int ballsToFire, boolean isBlue) {
        this(follower, targetPose, ballsToFire, -1, isBlue);
    }

    /** Explicit velocity override (ticks/s). */
    public MoveAndShoot(Follower follower, Pose targetPose, int ballsToFire,
                            double overrideVelocity, boolean isBlue) {
        this.follower         = follower;
        this.targetPose       = targetPose;
        this.ballsToFire      = ballsToFire;
        this.overrideVelocity = overrideVelocity;
        this.isBlue           = isBlue;
        addRequirements(robot.flywheel, robot.conveyor);
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        shotsFired  = 0;
        spinUpStart = System.currentTimeMillis();
        state       = State.DRIVING_AND_SPINNING;

        // Spin up immediately
        spinUpFlywheel();

        // Start the path — robot begins moving right away
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
                .setTimeoutConstraint(300)
                .build();

        follower.followPath(path, 1.0, true);
    }

    @Override
    public void execute() {
        // Always keep flywheel target fresh as distance changes mid-drive
        spinUpFlywheel();

        switch (state) {

            case DRIVING_AND_SPINNING:
                boolean flywheelReady = robot.flywheel.atTarget();
                boolean timedOut      = System.currentTimeMillis() - spinUpStart > SPIN_UP_TIMEOUT_MS;

                // Fire as soon as flywheel is ready — wait for path to end
                if (flywheelReady || timedOut && !follower.isBusy()) {
                    robot.conveyor.feed();
                    state = State.FEEDING;
                }
                break;

            case FEEDING:
                // Keep flywheel happy during feeding
                if (robot.flywheel.ballDetected()) {
                    robot.conveyor.stop();
                    shotsFired++;

                    if (shotsFired < ballsToFire) {
                        // Brief recovery window — reopen conveyor on next flywheel-ready tick
                        state       = State.DRIVING_AND_SPINNING;
                        spinUpStart = System.currentTimeMillis();
                    } else {
                        robot.flywheel.off();
                        state = State.DONE;
                    }
                }
                break;

            case DONE:
                // Path may still be running — let it finish naturally
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // Done when all balls are fired AND the path has completed
        return state == State.DONE && !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        robot.conveyor.stop();
        if (interrupted) {
            robot.flywheel.off();
        }
        // If not interrupted, leave flywheel running so the next shot
        // can fire faster — caller can .off() it when ready
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