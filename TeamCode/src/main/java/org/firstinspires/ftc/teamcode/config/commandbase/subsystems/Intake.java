package org.firstinspires.ftc.teamcode.config.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.config.globals.Constants.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * Intake subsystem — controls the intake roller motor.
 *<p>
 * This subsystem only manages the intake motor itself. Coordinating intake
 * with the conveyor during feeding is handled at the command level
 * (see commands/ package, e.g. SetIntake and intakePath in Auton).
 *<p>
 * Usage:
 *   robot.intake.forward();                        // start intake
 *   robot.intake.reverse();                        // reverse to eject
 *   robot.intake.stop();                           // stop
 *   robot.intake.toggle();                         // toggle forward/stop
 *   new SetIntake(Intake.MotorState.FORWARD)       // command-based version
 */
public class Intake extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    public enum MotorState { FORWARD, REVERSE, STOP }

    public static MotorState motorState = MotorState.STOP;

    // ─── Public API ───────────────────────────────────────────────────────────

    public void setIntake(MotorState state) {
        motorState = state;
        switch (state) {
            case FORWARD:
                robot.intakeMotor.set(-INTAKE_FORWARD_SPEED); // negative = intake direction
                break;
            case REVERSE:
                robot.intakeMotor.set(INTAKE_REVERSE_SPEED);
                break;
            case STOP:
            default:
                robot.intakeMotor.set(0);
                break;
        }
    }

    public void forward()  { setIntake(MotorState.FORWARD); }
    public void reverse()  { setIntake(MotorState.REVERSE); }
    public void stop()     { setIntake(MotorState.STOP); }

    public void toggle() {
        setIntake(motorState == MotorState.FORWARD ? MotorState.STOP : MotorState.FORWARD);
    }

    /**
     * Ball detection via velocity drop on the intake encoder.
     * Returns true on the loop cycle a ball was detected.
     */
    private double prevIntakeVel = -1.0;

    public boolean ballDetected() {
        double current = robot.intakeMotor.get(); // using power as proxy; swap for getVelocity() if encoder is attached
        if (prevIntakeVel < 0) {
            prevIntakeVel = current;
            return false;
        }
        if ((prevIntakeVel - current) > BALL_DETECTION_THRESHOLD) {
            prevIntakeVel = current;
            return true;
        }
        prevIntakeVel = current;
        return false;
    }

    @Override
    public void periodic() { }
}