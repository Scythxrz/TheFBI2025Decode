package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

/**
 * Instant command that sets the flywheel to a specific velocity.
 * Usage:
 *   new SpinFlywheel(1800)                   // exact ticks/s
 *   new SpinFlywheel(distanceInches, true)   // distance-based lookup
 */
public class SpinFlywheel extends InstantCommand {

    /** Set flywheel to an exact velocity in ticks/second. */
    public SpinFlywheel(double ticksPerSecond) {
        super(
                () -> Robot.getInstance().flywheel.setVelocity(ticksPerSecond),
                Robot.getInstance().flywheel
        );
    }

    /**
     * Set flywheel velocity from the shooter lookup table given a field distance.
     * @param distanceInches distance from robot to goal in inches
     * @param useDistanceLUT pass true to use the LUT (false just calls off())
     */
    public SpinFlywheel(double distanceInches, boolean useDistanceLUT) {
        super(
                () -> {
                    Flywheel fw = Robot.getInstance().flywheel;
                    if (useDistanceLUT) {
                        fw.setVelocityForDistance(distanceInches);
                    } else {
                        fw.off();
                    }
                },
                Robot.getInstance().flywheel
        );
    }
}