package org.firstinspires.ftc.teamcode.config.globals;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Central constants file — all tunable values live here.
 * @Configurable exposes every public static field to FTC Dashboard for live tuning.
 */
@Configurable
public class Constants {

    // ─── Op Mode Type ─────────────────────────────────────────────────────────
    public enum OpModeType { AUTO, TELEOP }
    public enum AllianceColor {
        BLUE(1), RED(-1);
        private final int multiplier;
        AllianceColor(int m) { multiplier = m; }
        public int getMultiplier() { return multiplier; }
    }

    public static OpModeType  OP_MODE_TYPE;
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;

    // ─── Voltage ──────────────────────────────────────────────────────────────
    public static double VOLTAGE_SENSOR_POLLING_RATE = 5.0;  // Hz
    public static double NOMINAL_VOLTAGE             = 13.5; // Volts

    // ─── Shooter / Flywheel ───────────────────────────────────────────────────
    /**
     * PIDF gains for the software flywheel velocity controller (SolversLib PIDFController).
     * All four values are live-tunable on FTC Dashboard via @Configurable.
     *
     * Tuning order:
     *   1. Set kP/kI/kD = 0. Raise kF until the flywheel gets close to target
     *      velocity on its own (~1/MAX_SHOOTER_VELOCITY is a starting point).
     *   2. Raise kP until steady-state error disappears without oscillation.
     *   3. Add a small kD only if the flywheel overshoots noticeably on spin-up.
     *   4. kI is rarely needed for flywheels — leave at 0 unless there is
     *      persistent steady-state error that kP alone cannot correct.
     *
     *   kF  — feedforward: scales target velocity directly into motor power.
     *          Start near 1.0 / MAX_SHOOTER_VELOCITY ≈ 0.000417.
     *   kP  — proportional: corrects remaining error after kF.
     *   kI  — integral: eliminates small persistent offset (use sparingly).
     *   kD  — derivative: dampens overshoot on spin-up.
     */
    public static double SHOOTER_KF = 0.0004;  // ≈ 1 / 2400
    public static double SHOOTER_KP = 0.001;
    public static double SHOOTER_KI = 0.0;
    public static double SHOOTER_KD = 0.0;

    // How close the flywheel needs to be to targetVelocity before feeding starts.
    // Wider = shoots sooner at the cost of slightly reduced accuracy.
    // If the flywheel consistently levels off 100+ ticks below target, raise this.
    public static double SHOOTER_READY_TOLERANCE    = 150;  // ticks/s

    // Minimum velocity DROP in a single loop tick to count as a ball passing through.
    // Must be large enough to ignore normal flywheel noise (~5-15 ticks/s) but
    // small enough to catch real ball events (~80-150 ticks/s drop).
    // Do NOT tie this to SHOOTER_READY_TOLERANCE — they serve different purposes.
    public static double SHOOTER_BALL_DROP_THRESHOLD  = 100;   // ticks/s

    // After counting a ball, ignore velocity drops for this long.
    // Should be long enough for the flywheel to recover (~100-300ms typically).
    // Too short → double-counts one ball. Too long → misses rapid consecutive balls.
    public static long   BALL_DETECTION_COOLDOWN_MS  = 150;  // ms
    public static double MAX_SHOOTER_VELOCITY        = 2400; // ticks/s
    public static final double MIN_COMP_FACTOR   = 0.2;   // factor at nominal voltage
    public static final double MAX_COMP_FACTOR   = 0.7;   // factor at low voltage
    public static final double QUAD_A            = 0.128; // (MAX - MIN) / (NOMINAL - LOW_V)^2

    // Shooter lookup table — {distance_inches, target_velocity_ticks_per_sec}
    // Matches the table from FBI2025 Shooter.java / FBI2025 Flywheel.java
    public static final double[][] SHOOTER_LUT = {
            {65, 1400},
            {75, 1500},
            {85, 1550},
            {95, 1650},
            {100, 1750},
            {113, 1950},
            {126, 2350},
            {133, 2400},
    };

    // ─── Intake ───────────────────────────────────────────────────────────────
    public static double INTAKE_FORWARD_SPEED  =  1.0;
    public static double INTAKE_REVERSE_SPEED  =  1.0;
    public static double KICKER_FORWARD_POWER  = -1.0;

    // Ball detection: drop in intake encoder velocity that signals a ball was intaked
    public static double BALL_DETECTION_THRESHOLD = 600; // ticks/s

    // ─── Conveyor ─────────────────────────────────────────────────────────────
    public static double CONVEYOR_CLOSE_SPEED = 1;
    public static double CONVEYOR_INTAKE_SPEED = 1.0;
    public static double CONVEYOR_FAR_SPEED = 0.4;
    public static double CONVEYOR_REVERSE_SPEED = -1.0;

    // Delay after stopper opens before conveyor starts feeding (ms)
    public static long CONVEYOR_FEED_DELAY_MS = 250;

    // ─── Gate / Stopper servo ─────────────────────────────────────────────────
    public static double STOPPER_OPEN   = 1.0;
    public static double STOPPER_CLOSED = 0.0;

    // ─── Heading aim tolerances (TeleOp aimbot) ───────────────────────────────
    public static double AIM_ANGLE_TOLERANCE   = 0.1;  // radians
    public static double AIM_ANGULAR_VEL_MAX   = 0.3;  // rad/s
    public static long   AIM_SETTLE_TIME_MS    = 120;  // ms heading must be stable before firing

    // Paced fire: time between shots (ms)
    /** How long the conveyor runs once the flywheel is at speed. Tune on the field (e.g. 3 balls ~1500 ms). */
    public static long SHOOT_FEED_TIME_MS = 750;
    /** Max ms to wait for flywheel recovery between paced balls before firing anyway. */
    public static long PACED_RECOVERY_TIMEOUT_MS = 600;

    // ─── Goal pose [x_inches, y_inches] (Pedro field coords) ─────────────────
    // Blue alliance default — mirrored for Red in TeleOp initialize()
    public static double[] GOAL_POSE_BLUE = {2.0,  142.0};
    public static double[] GOAL_POSE_RED  = {142.0, 142.0};
}