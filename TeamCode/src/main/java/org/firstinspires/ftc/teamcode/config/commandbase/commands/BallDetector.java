package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import static org.firstinspires.ftc.teamcode.config.globals.Constants.*;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * BallDetector — fresh-per-command, self-contained ball passage detector.
 *
 * Instantiate a new one in each command's initialize() so there is zero
 * shared state between consecutive shooting commands.
 *
 * Algorithm:
 *   WAITING_FOR_PEAK  — tracks rising velocity; waits for it to stabilize
 *                       (3 ticks within 30 ticks/s of peak) before watching.
 *                       Prevents false triggers during spin-up.
 *   WATCHING          — watches for a drop > SHOOTER_BALL_DROP_THRESHOLD from
 *                       the tracked peak. Returns true exactly once per ball.
 *   COOLDOWN          — blinds detector for BALL_DETECTION_COOLDOWN_MS after a
 *                       detection. On exit resets peak to current velocity so
 *                       the next detection is relative to the actual recovered
 *                       level — critical for shoot-on-move where the flywheel
 *                       may settle lower than the original target.
 */
public class BallDetector {

    private final Robot robot = Robot.getInstance();

    private enum Phase { WAITING_FOR_PEAK, WATCHING, COOLDOWN }
    private Phase  phase         = Phase.WAITING_FOR_PEAK;
    private double peak          = 0;
    private long   cooldownStart = 0;

    private static final double STABLE_BAND  = 30; // ticks/s — how tight "stable" is
    private static final int    STABLE_TICKS = 3;  // consecutive ticks needed to confirm stable
    private int stableTicker = 0;

    /** Call every loop tick while the conveyor is feeding. Returns true once per ball. */
    public boolean update() {
        double current = robot.flywheel.getVelocity();

        switch (phase) {

            case WAITING_FOR_PEAK:
                if (current > peak) {
                    peak         = current;
                    stableTicker = 0;
                } else if (Math.abs(current - peak) <= STABLE_BAND) {
                    if (++stableTicker >= STABLE_TICKS) phase = Phase.WATCHING;
                } else {
                    stableTicker = 0; // dropped before stabilising — noise
                }
                return false;

            case WATCHING:
                if (current > peak) peak = current; // track recovery peaks upward
                if ((peak - current) > SHOOTER_BALL_DROP_THRESHOLD) {
                    cooldownStart = System.currentTimeMillis();
                    phase         = Phase.COOLDOWN;
                    return true;
                }
                return false;

            case COOLDOWN:
                if (System.currentTimeMillis() - cooldownStart >= BALL_DETECTION_COOLDOWN_MS) {
                    // Reset to current recovered velocity — not the old pre-ball peak
                    peak         = current;
                    stableTicker = 0;
                    phase        = Phase.WAITING_FOR_PEAK;
                }
                return false;

            default:
                return false;
        }
    }
}