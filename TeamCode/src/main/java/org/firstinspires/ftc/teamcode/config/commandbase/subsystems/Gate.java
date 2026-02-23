package org.firstinspires.ftc.teamcode.config.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.config.globals.Constants.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * Gate subsystem — wraps the stopper servo that holds balls in the conveyor
 * until the flywheel is up to speed.
 */
public class Gate extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    public void open() {
        robot.stopperServo.setPosition(STOPPER_OPEN);
    }

    public void close() {
        robot.stopperServo.setPosition(STOPPER_CLOSED);
    }

    @Override
    public void periodic() { }
}