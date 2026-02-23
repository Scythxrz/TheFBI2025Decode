package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Conveyor;
import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.config.globals.Constants;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * Full launch sequence:
 *   1. Waits until the flywheel is at target velocity AND heading is settled.
 *   2. Opens the conveyor to feed a ball through.
 *   3. Detects the ball passing (velocity drop) and stops feeding.
 *
 * This is the command-based equivalent of FBI2025's handleGamepad2() shooting logic.
 * Bind it to a button with whenHeld() or whenPressed().
 *
 * Usage:
 *   operator.getGamepadButton(GamepadKeys.Button.Y)
 *       .whileActiveContinuous(new LaunchSequence(() -> distanceToGoal, () -> headingError));
 */
public class LaunchSequence extends CommandBase {

    public interface DoubleSupplier { double get(); }

    private final DoubleSupplier distanceSupplier;
    private final DoubleSupplier headingErrorSupplier;

    private final Robot robot = Robot.getInstance();
    private final Flywheel flywheel;
    private final Conveyor conveyor;

    private boolean feeding = false;
    private long shotTime   = 0;

    /**
     * @param distanceSupplier      lambda that returns current distance to goal in inches
     * @param headingErrorSupplier  lambda that returns current heading error in radians
     */
    public LaunchSequence(DoubleSupplier distanceSupplier, DoubleSupplier headingErrorSupplier) {
        this.distanceSupplier     = distanceSupplier;
        this.headingErrorSupplier = headingErrorSupplier;

        flywheel = robot.flywheel;
        conveyor = robot.conveyor;

        addRequirements(flywheel, conveyor);
    }

    @Override
    public void initialize() {
        feeding  = false;
        shotTime = 0;
        // Spin up to the correct velocity for current distance
        flywheel.setVelocityForDistance(distanceSupplier.get());
    }

    @Override
    public void execute() {
        // Keep updating flywheel target as robot moves
        flywheel.setVelocityForDistance(distanceSupplier.get());

        boolean flywheelReady  = flywheel.atTarget();
        boolean headingSettled = Math.abs(headingErrorSupplier.get()) < Constants.AIM_ANGLE_TOLERANCE;

        if (flywheelReady && headingSettled) {
            if (!feeding) {
                feeding  = true;
                shotTime = System.currentTimeMillis();
                conveyor.feed();
            }
        } else {
            if (feeding) {
                // Heading or flywheel lost — stop feeding and wait for recovery
                conveyor.stop();
                feeding = false;
            }
        }

        // Stop feeding after a ball has passed (detected by velocity drop)
        if (feeding && flywheel.ballDetected()) {
            conveyor.stop();
            feeding = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        flywheel.off();
        feeding = false;
    }

    @Override
    public boolean isFinished() {
        // Run until the button is released (whileActiveContinuous) or cancelled
        return false;
    }
}