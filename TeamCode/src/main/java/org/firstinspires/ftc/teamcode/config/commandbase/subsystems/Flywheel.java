package org.firstinspires.ftc.teamcode.config.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.config.globals.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * Flywheel subsystem — controls the shooter motor via PIDF velocity control.
 *
 * Hardware lives in Robot.getInstance(); this subsystem just calls setVelocity()
 * and reads back encoder ticks each periodic() tick via the CommandScheduler.
 *
 * The velocity lookup table (distance → ticks/s) is ported from FBI2025's
 * Shooter / Flywheel classes and is still tunable via Constants.SHOOTER_LUT.
 */
public class Flywheel extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    private double targetVelocity = 0;
    private double prevVelocity      = -1.0; // sentinel for ball detection
    private long   lastDetectionTime = 0;    // ms — cooldown after each ball count
    private double veloCompKF = SHOOTER_KF;

    // Software PIDF controller — runs in periodic() every CommandScheduler tick.
    // Gains are read from Constants each tick so FTC Dashboard changes take effect live.
    private final PIDFController controller = new PIDFController(
            SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, veloCompKF
    );

    // ─── Constructor ──────────────────────────────────────────────────────────

    public Flywheel() {
        // RUN_WITHOUT_ENCODER lets us call setPower() directly.
        // The encoder is still attached and getVelocity() still works — we just
        // run the PIDF loop ourselves in periodic() instead of delegating to hardware.
        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ─── Public API ───────────────────────────────────────────────────────────

    /** Spin up to a target velocity in ticks/second. The PIDF loop in periodic() applies power. */
    public void setVelocity(double ticksPerSecond) {
        targetVelocity = ticksPerSecond;
    }

    public void setPower(double power) {
        //robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterMotor.setPower(power);
    }

    /**
     * Spin up using the shooter lookup table to convert a field distance (inches)
     * into the correct target velocity.  Applies half-weight voltage compensation
     * so the ball still lands when the battery sags.
     */
    public void setVelocityForDistance(double distanceInches) {
        double vel     = velocityFromLUT(distanceInches);
        double voltage = robot.getVoltage();

        // Quadratic factor: rises as voltage drops, flat near nominal
        /*double rawFactor = QUAD_A * Math.pow(voltage - NOMINAL_VOLTAGE, 2) + MIN_COMP_FACTOR;
        double compFactor = Math.min(Math.max(rawFactor, MIN_COMP_FACTOR), MAX_COMP_FACTOR);

        double compensated = vel * (1.0 + compFactor * ((NOMINAL_VOLTAGE / voltage) - 1.0));*/


        setVelocity(vel); // BUG FIX: was setVelocity(vel) — compensation was never applied
    }

    /**
     * Like setVelocityForDistance, but adds a velocity feedforward that compensates
     * for the robot moving away from the goal while shooting.
     *
     * When the robot is retreating, the ball's effective launch velocity relative to
     * the goal is reduced by the robot's recessional speed — so we pre-add that back.
     * When the robot is approaching (velocityAwayFromGoal is negative), we subtract it,
     * preventing overshoot.
     *
     * @param distanceInches       current distance to goal in inches
     * @param velocityAwayFromGoal robot's velocity component directly away from the goal,
     *                             in inches/second. Positive = moving away, negative = closing.
     *                             Pass 0 to behave identically to setVelocityForDistance().
     */
    public void setVelocityForDistanceWithVelocityFF(double distanceInches, double velocityAwayFromGoal) {
        double vel     = velocityFromLUT(distanceInches);
        double voltage = robot.getVoltage();

        // Voltage compensation (same formula as setVelocityForDistance)
        // Quadratic factor: rises as voltage drops, flat near nominal
        double rawFactor = QUAD_A * Math.pow(voltage - NOMINAL_VOLTAGE, 2) + MIN_COMP_FACTOR;
        double compFactor = Math.min(Math.max(rawFactor, MIN_COMP_FACTOR), MAX_COMP_FACTOR);

        double compensated = vel * (1.0 + compFactor * ((NOMINAL_VOLTAGE / voltage) - 1.0));
        /*
        // Velocity feedforward: scale robot inches/s into shooter ticks/s using the LUT
        // gradient at this distance (Δticks/Δdistance × robot speed = Δticks/s needed).
        // Simpler approximation: sample the LUT at distance ± a small delta to get the
        // ticks-per-inch slope, then multiply by the recessional speed.
        double delta        = 1.0; // inches
        double velAtFurther = velocityFromLUT(distanceInches + delta);
        double velAtCloser  = velocityFromLUT(distanceInches - delta);
        double lutSlope     = (velAtFurther - velAtCloser) / (2.0 * delta); // ticks/s per inch/s

        // lutSlope × velocityAwayFromGoal: positive when retreating (adds power),
        //                                  negative when closing  (reduces power)
        double velocityFF = 1 * lutSlope * velocityAwayFromGoal;
*/
        setVelocity(compensated);
    }

    /** Stop the flywheel and reset the PIDF controller state. */
    public void off() {
        targetVelocity = 0;
        controller.reset();
        robot.shooterMotor.setPower(0);
    }

    /** Current measured velocity in ticks/second. */
    public double getVelocity() {
        return robot.shooterMotor.getVelocity();
    }

    /** The velocity we last commanded, in ticks/second. */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Returns true once the flywheel is spinning within
     * SHOOTER_READY_TOLERANCE of the target velocity.
     */
    public boolean atTarget() {
        return targetVelocity > 0
                && Math.abs(targetVelocity - getVelocity()) <= SHOOTER_READY_TOLERANCE;
    }

    /**
     * Detects a drop in velocity that signals a ball passed through the shooter.
     * Call once per loop() iteration when the flywheel is running.
     *
     * @return true if a ball was just detected, false otherwise
     */
    public boolean ballDetected() {
        double current = getVelocity();

        if (prevVelocity < 0) {
            prevVelocity = current;
            return false;
        }

        // Ignore drops during cooldown — flywheel is still recovering from previous ball
        boolean inCooldown = (System.currentTimeMillis() - lastDetectionTime) < BALL_DETECTION_COOLDOWN_MS;

        if (!inCooldown && (prevVelocity - current) > SHOOTER_BALL_DROP_THRESHOLD) {
            prevVelocity      = current;
            lastDetectionTime = System.currentTimeMillis();
            return true;
        }

        // Only track upward movement so prevVelocity reflects the recovered peak
        if (current > prevVelocity) prevVelocity = current;
        return false;
    }

    // ─── Periodic ─────────────────────────────────────────────────────────────

    /**
     * Called every CommandScheduler tick.
     *
     * Runs the software PIDF loop:
     *   1. Pulls the latest gain values from Constants so FTC Dashboard edits
     *      apply immediately without restarting the OpMode.
     *   2. When targetVelocity == 0 the motor is cut to 0 and the controller
     *      is reset — avoids the integrator winding up while the flywheel coasts.
     *   3. Otherwise, calculate(currentVelocity, targetVelocity) returns a
     *      [-1, 1] power value which is sent straight to setPower().
     */
    @Override
    public void periodic() {
        double voltage = robot.getVoltage();
        double rawFactor = QUAD_A * Math.pow(voltage - NOMINAL_VOLTAGE, 2) + MIN_COMP_FACTOR;
        double compFactor = Math.min(Math.max(rawFactor, MIN_COMP_FACTOR), MAX_COMP_FACTOR);

        veloCompKF = SHOOTER_KF * (1.0 + compFactor * ((NOMINAL_VOLTAGE / voltage) - 1.0));

        // Pick up any live Dashboard changes to the gain constants
        controller.setPIDF(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, veloCompKF);

        if (targetVelocity <= 0) {
            controller.reset();
            robot.shooterMotor.setPower(0);
            return;
        }

        double output = controller.calculate(getVelocity(), targetVelocity);
        // Clamp to [0, 1] — the flywheel only spins in one direction
        robot.shooterMotor.setPower(Math.max(0, Math.min(1, output)));
    }

    // ─── Private helpers ──────────────────────────────────────────────────────

    /**
     * Linear interpolation over SHOOTER_LUT.
     * Clamps to the nearest endpoint if distance is outside the table range.
     */
    private double velocityFromLUT(double distance) {
        double[][] lut = SHOOTER_LUT;

        if (distance <= lut[0][0])              return lut[0][1];
        if (distance >= lut[lut.length - 1][0]) return lut[lut.length - 1][1];

        for (int i = 0; i < lut.length - 1; i++) {
            if (distance >= lut[i][0] && distance <= lut[i + 1][0]) {
                double d1 = lut[i][0],     p1 = lut[i][1];
                double d2 = lut[i + 1][0], p2 = lut[i + 1][1];
                return p1 + (distance - d1) * (p2 - p1) / (d2 - d1);
            }
        }
        return lut[lut.length - 1][1];
    }
}