package org.firstinspires.ftc.teamcode.config.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.config.globals.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * Flywheel subsystem — controls the shooter motor via PIDF velocity control.
 *
 * Hardware lives in Robot.getInstance(); this subsystem just calls setVelocity()
 * and reads back encoder ticks each periodic() tick via the CommandScheduler.
 * Shooter / Flywheel classes and is still tunable via Constants.SHOOTER_LUT.
 */
public class Flywheel extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    private double targetVelocity = 0;
    private double prevVelocity   = -1.0; // sentinel for ball detection

    // ─── Constructor ──────────────────────────────────────────────────────────

    public Flywheel() {
        // Apply PIDF coefficients to the motor's RUN_USING_ENCODER mode
        robot.shooterMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                SHOOTER_PIDF
        );
        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ─── Public API ───────────────────────────────────────────────────────────

    /** Spin up to an exact velocity in ticks/second (no voltage compensation). */
    public void setVelocity(double ticksPerSecond) {
        targetVelocity = ticksPerSecond;
        robot.shooterMotor.setVelocity(ticksPerSecond);
    }

    /**
     * Spin up using the shooter lookup table to convert a field distance (inches)
     * into the correct target velocity.  Applies half-weight voltage compensation
     * so the ball still lands when the battery sags.
     */
    public void setVelocityForDistance(double distanceInches) {
        double vel = velocityFromLUT(distanceInches);
        double voltage = robot.getVoltage();
        // Partial voltage compensation (matches FBI2025 formula)
        double compensated = vel * (1.0 + VOLTAGE_COMP_FACTOR * ((NOMINAL_VOLTAGE / voltage) - 1.0));
        setVelocity(compensated);
    }

    /** Stop the flywheel. */
    public void off() {
        targetVelocity = 0;
        robot.shooterMotor.setVelocity(0);
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
     * SHOOTER_AT_TARGET_TOLERANCE of the target velocity.
     */
    public boolean atTarget() {
        return targetVelocity > 0
                && Math.abs(targetVelocity - getVelocity()) <= SHOOTER_AT_TARGET_TOLERANCE;
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

        if ((prevVelocity - current) > SHOOTER_AT_TARGET_TOLERANCE) {
            prevVelocity = current;
            return true;
        }

        if (current > prevVelocity) prevVelocity = current;
        return false;
    }

    // ─── Periodic ─────────────────────────────────────────────────────────────
    // No continuous update needed here; velocity is set imperatively.
    // If you add a software PIDF loop (like Decode 2026's Launcher), put it here.
    @Override
    public void periodic() { }

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