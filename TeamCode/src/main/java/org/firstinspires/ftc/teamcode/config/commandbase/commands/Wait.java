package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

/**
 * Wait — a no-op command that does nothing and never finishes on its own.
 *<p>
 * Always pair with .withTimeout(ms) to use as a timed delay in a sequence.
 *<p>
 * Usage:
 *   new Wait().withTimeout(1000)   // pause auto for 1 second
 *<p>
 * The helper method in Auton wraps this for convenience:
 *   wait(1000.0)   // equivalent shorthand inside Auton
 */
public class Wait extends CommandBase {
}
