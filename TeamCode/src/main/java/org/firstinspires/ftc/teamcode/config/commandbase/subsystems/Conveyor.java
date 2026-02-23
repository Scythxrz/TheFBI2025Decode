package org.firstinspires.ftc.teamcode.config.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.config.globals.Constants.*;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * Conveyor subsystem — controls the belt motor and gate servo that
 * feed balls from the intake into the flywheel.
 *
 * Feeding sequence:
 *   1. Gate servo opens (STOPPER_OPEN)
 *   2. After CONVEYOR_FEED_DELAY_MS, belt motor spins
 *   3. Balls roll into the flywheel
 *
 * Stopping:
 *   Belt off, gate closes (STOPPER_CLOSED)
 */
public class Conveyor extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    public enum ConveyorState { STOPPED, FEEDING, REVERSING }

    private ConveyorState state         = ConveyorState.STOPPED;
    private final Timer   feedTimer     = new Timer();
    private boolean       feedTimerStarted = false;

    // ─── Public API ───────────────────────────────────────────────────────────

    /**
     * Open the gate and start feeding balls into the flywheel.
     * Call once to begin — periodic() handles the belt delay internally.
     */
    public void feed() {
        if (state != ConveyorState.FEEDING) {
            state            = ConveyorState.FEEDING;
            feedTimerStarted = false;
            feedTimer.resetTimer();
            robot.stopperServo.setPosition(STOPPER_OPEN);
        }
    }

    /** Reverse the belt to clear jams. Gate stays closed. */
    public void reverse() {
        state            = ConveyorState.REVERSING;
        feedTimerStarted = false;
        robot.stopperServo.setPosition(STOPPER_CLOSED);
        robot.conveyorMotor.set(CONVEYOR_REVERSE_SPEED);
    }

    /** Stop belt and close the gate. */
    public void stop() {
        state            = ConveyorState.STOPPED;
        feedTimerStarted = false;
        robot.conveyorMotor.set(0);
        robot.stopperServo.setPosition(STOPPER_CLOSED);
    }

    public ConveyorState getState() { return state; }

    // ─── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        if (state == ConveyorState.FEEDING) {
            if (!feedTimerStarted) {
                feedTimer.resetTimer();
                feedTimerStarted = true;
            }
            // Brief delay after gate opens before spinning belt,
            // so the ball is fully clear of the gate before it starts moving
            if (feedTimer.getElapsedTime() > CONVEYOR_FEED_DELAY_MS) {
                robot.conveyorMotor.set(CONVEYOR_FORWARD_SPEED);
            }
        }
    }
}