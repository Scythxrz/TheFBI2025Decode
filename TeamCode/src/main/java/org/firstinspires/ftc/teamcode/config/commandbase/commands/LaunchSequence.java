package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Conveyor;
import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.config.globals.Constants;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * LaunchSequence — TeleOp shooting command, bound to a held button.
 *<p>
 * Runs until the button is released (isFinished always returns false).
 * The operator holds the button; this command fires as many balls as come
 * through while the button is held and conditions are met.
 <p>
 * ── Firing modes ──────────────────────────────────────────────────────────────
 *<p>
 *   RAPID (default — bind to Y)
 *     Gate opens once flywheel + heading are ready and stays open.
 *     Balls pass through continuously with no pause between shots.
 *     If heading or flywheel drops out, the gate closes until they recover.
 *     Use for close-range where the flywheel recovers between balls fast enough.
 *<p>
 *   PACED (bind to A or RB)
 *     Gate closes after each detected ball and waits for flywheel to recover
 *     back to target speed before reopening.
 *     Use for far shots where velocity sag between balls would hurt accuracy.
 *<p>
 * ── No ball-count timeout in TeleOp ──────────────────────────────────────────
 *   Unlike auto, there's no ball-count limit or global timeout here — the
 *   operator controls duration by holding/releasing the button. The command
 *   simply keeps firing until the button is released.
 *<p>
 * ── Usage (in TeleOp initialize()) ───────────────────────────────────────────
 *   // Rapid — hold Y
 *   operator.getGamepadButton(GamepadKeys.Button.Y).whileActiveContinuous(
 *       new LaunchSequence(() -> distanceToGoal, () -> headingError)
 *   );
 *<p>
 *   // Paced — hold A
 *   operator.getGamepadButton(GamepadKeys.Button.A).whileActiveContinuous(
 *       new LaunchSequence(() -> distanceToGoal, () -> headingError, FiringMode.PACED)
 *   );
 */
public class LaunchSequence extends CommandBase {

    public enum FiringMode { RAPID, PACED }

    public interface DoubleSupplier { double get(); }

    private enum State { WAITING, FEEDING, RECOVERING }
    private State state;

    private final DoubleSupplier distanceSupplier;
    private final DoubleSupplier headingErrorSupplier;
    private final FiringMode     firingMode;

    private final Robot    robot;
    private final Flywheel flywheel;
    private final Conveyor conveyor;

    private long spinUpStart = 0;
    private static final long SPIN_UP_TIMEOUT_MS = 2000;

    // ─── Constructors ─────────────────────────────────────────────────────────

    /** Rapid fire (default). */
    public LaunchSequence(DoubleSupplier distanceSupplier, DoubleSupplier headingErrorSupplier) {
        this(distanceSupplier, headingErrorSupplier, FiringMode.RAPID);
    }

    /** Explicit firing mode. */
    public LaunchSequence(DoubleSupplier distanceSupplier, DoubleSupplier headingErrorSupplier,
                          FiringMode firingMode) {
        this.distanceSupplier     = distanceSupplier;
        this.headingErrorSupplier = headingErrorSupplier;
        this.firingMode           = firingMode;

        robot    = Robot.getInstance();
        flywheel = robot.flywheel;
        conveyor = robot.conveyor;

        addRequirements(flywheel, conveyor);
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        state        = State.WAITING;
        spinUpStart  = System.currentTimeMillis();
        flywheel.setVelocityForDistance(distanceSupplier.get());
    }

    @Override
    public void execute() {
        // Always keep flywheel target fresh
        flywheel.setVelocityForDistance(distanceSupplier.get());

        boolean flywheelReady  = flywheel.atTarget();
        boolean headingSettled = Math.abs(headingErrorSupplier.get()) < Constants.AIM_ANGLE_TOLERANCE;
        boolean spinTimedOut   = System.currentTimeMillis() - spinUpStart > SPIN_UP_TIMEOUT_MS;
        boolean readyToFire    = (flywheelReady && headingSettled) || spinTimedOut;

        switch (state) {

            case WAITING:
                if (readyToFire) {
                    conveyor.feed();
                    state = State.FEEDING;
                }
                break;

            case FEEDING:
                if (!readyToFire && !spinTimedOut) {
                    // Lost heading or flywheel — close gate and wait for recovery
                    conveyor.stop();
                    spinUpStart = System.currentTimeMillis();
                    state = State.WAITING;
                    break;
                }

                if (flywheel.ballDetected()) {
                    if (firingMode == FiringMode.PACED) {
                        // Close gate, wait for flywheel recovery before next ball
                        conveyor.stop();
                        spinUpStart = System.currentTimeMillis();
                        state = State.RECOVERING;
                    }
                    // RAPID: gate stays open — next ball feeds immediately
                }
                break;

            case RECOVERING:
                if (flywheel.atTarget() || System.currentTimeMillis() - spinUpStart > SPIN_UP_TIMEOUT_MS) {
                    // Flywheel recovered — reopen for next ball
                    if (readyToFire) {
                        conveyor.feed();
                        state = State.FEEDING;
                    } else {
                        state = State.WAITING;
                    }
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        flywheel.off();
    }

    @Override
    public boolean isFinished() {
        // Never finishes on its own — button release cancels it
        return false;
    }
}