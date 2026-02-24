package org.firstinspires.ftc.teamcode.config.globals;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Central constants file — all tunable values live here.
 * @Configurable exposes every public static field to Panels for live tuning.
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
    public static double NOMINAL_VOLTAGE             = 13.0; // Volts

    // ─── Shooter / Flywheel ───────────────────────────────────────────────────
    /**
     * PIDF for the shooter motor running in RUN_USING_ENCODER mode.
     * Start with these values and tune kP / kF on FTC Dashboard.
     *   kP  — corrects steady-state error
     *   kI  — usually left at 0 for flywheels
     *   kD  — usually left at 0 for flywheels
     *   kF  — feedforward (~11 is a common starting point for NeveRest / GoBILDA)
     */
    public static PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(40, 0, 0, 11);

    public static double SHOOTER_AT_TARGET_TOLERANCE = 60;   // ticks/s — "close enough" to fire
    public static double MAX_SHOOTER_VELOCITY        = 2600; // ticks/s

    // Velocity compensation multiplier (scales for voltage sag)
    // Formula used: velocity * (NOMINAL_VOLTAGE / measuredVoltage) * VOLTAGE_COMP_FACTOR
    public static double VOLTAGE_COMP_FACTOR = 0.5;

    // Shooter lookup table — {distance_inches, target_velocity_ticks_per_sec}
    // Add or adjust rows to tune velocity vs. distance for your field setup.
    // Used by Flywheel.setVelocityForDistance() and WindUpAndDrive's LUT constructors.
    public static final double[][] SHOOTER_LUT = {
            {50.45,   1650},
            {60.2258, 1700},
            {65.14,   1725},
            {70.04,   1750},
            {75.6015, 1750},
            {80.3616, 1840},
            {85.6286, 1875},
            {90.393,  1925},
            {95.1473, 1975},
            {100.56,  2025},
            {105.4738,2050},
            {111.2026,2100},
            {114.2744,2150},
            {120.4631,2200},
            {124.8829,2275},
            {130.4457,2350},
            {134.4726,2375},
            {141.0453,2475}
    };

    // ─── Intake ───────────────────────────────────────────────────────────────
    public static double INTAKE_FORWARD_SPEED  =  1.0;
    public static double INTAKE_REVERSE_SPEED  =  0.5;
    public static double KICKER_FORWARD_POWER  = -1.0;

    // Ball detection: drop in intake encoder velocity that signals a ball was intaked
    public static double BALL_DETECTION_THRESHOLD = 600; // ticks/s

    // ─── Conveyor ─────────────────────────────────────────────────────────────
    public static double CONVEYOR_FORWARD_SPEED = 1.0;
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
    public static long PACED_SHOT_FEED_TIME_MS = 500;

    // ─── Goal pose [x_inches, y_inches] (Pedro field coords) ─────────────────
    // Blue alliance default — mirrored for Red in TeleOp initialize()
    public static double[] GOAL_POSE_BLUE = {2.0,  142.0};
    public static double[] GOAL_POSE_RED  = {142.0, 142.0};
}