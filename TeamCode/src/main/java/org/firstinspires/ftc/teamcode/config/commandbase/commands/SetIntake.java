package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * Instant command that sets the intake to a given MotorState.
 * Usage:  new SetIntake(Intake.MotorState.FORWARD)
 */
public class SetIntake extends InstantCommand {
    public SetIntake(Intake.MotorState state) {
        super(
                () -> Robot.getInstance().intake.setIntake(state),
                Robot.getInstance().intake
        );
    }
}