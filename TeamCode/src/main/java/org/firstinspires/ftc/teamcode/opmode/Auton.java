package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.globals.Constants.*;
import static org.firstinspires.ftc.teamcode.config.globals.Poses.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.config.commandbase.commands.DriveToPose;
import org.firstinspires.ftc.teamcode.config.commandbase.commands.MoveAndShoot;
import org.firstinspires.ftc.teamcode.config.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.config.commandbase.commands.ShootWhileMoving;
import org.firstinspires.ftc.teamcode.config.commandbase.commands.Wait;
import org.firstinspires.ftc.teamcode.config.commandbase.commands.WindUpAndDrive;
import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.globals.Poses;
import org.firstinspires.ftc.teamcode.config.globals.Robot;
import org.firstinspires.ftc.teamcode.config.globals.PedroConstants;

import java.io.FileWriter;
import java.io.IOException;

/**
 * Main Autonomous OpMode.
 *<p>
 * All poses come from Poses.java — nothing is hardcoded here.
 *<p>
 * Three helper methods cover every motion type:
 *<p>
 *   windUpAndDrive(to, vel, speed)
 *     → drives a path while flywheel spins up, conveyor closed
 *<p>
 *   shootWhileMoving(to, balls, vel)
 *     → drives a path while flywheel fires mid-path (assumes flywheel already warm)
 *<p>
 *   intakePath(mid, collect, speed)
 *     → drives to mid then slows to collect with intake running
 *<p>
 * Typical pattern for a full scoring cycle:
 *<p>
 *   windUpAndDrive(PGP_COLLECT, 1850, 0.6)    // collecting, flywheel warming
 *   shootWhileMoving(SCORE_CLOSE, 2, 1850)     // driving back, fires en route
 *<p>
 * Init-loop menu:
 *   DPAD up/down   — cycle sequence
 *   DPAD left      — toggle alliance (Blue / Red)
 *   DPAD right     — toggle start side (Close / Far)
 */
@Configurable
@Autonomous(name = "Auto", group = "AAAAuto", preselectTeleOp = "TeleOp")
public class Auton extends CommandOpMode {

    // ─── Robot & telemetry ────────────────────────────────────────────────────
    private final Robot robot = Robot.getInstance();
    private ElapsedTime loopTimer;
    private Follower follower;

    // ─── Menu ─────────────────────────────────────────────────────────────────
    private enum StartPos { CLOSE, FAR }
    private enum Sequence { CLOSE_18, CLOSE_15, FAR_9, FAR_9_GPP }

    private StartPos selectedStart    = StartPos.CLOSE;
    private Sequence selectedSequence = Sequence.CLOSE_18;
    private boolean  isBlue           = true;
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private boolean upLast, downLast, leftLast, rightLast;

    // ─── initialize() ─────────────────────────────────────────────────────────
    @Override
    public void initialize() {
        super.reset();
        OP_MODE_TYPE = OpModeType.AUTO;

        robot.init(hardwareMap);
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose());

        schedule(buildSequence());
    }

    // ─── initialize_loop() ────────────────────────────────────────────────────
    @Override
    public void initialize_loop() {
        boolean changed = false;

        if (gamepad1.dpad_up && !upLast) {
            selectedSequence = Sequence.values()[
                    (selectedSequence.ordinal() - 1 + Sequence.values().length) % Sequence.values().length];
            changed = true;
        }
        if (gamepad1.dpad_down && !downLast) {
            selectedSequence = Sequence.values()[
                    (selectedSequence.ordinal() + 1) % Sequence.values().length];
            changed = true;
        }
        if (gamepad1.dpad_left && !leftLast) {
            isBlue  = !isBlue;
            changed = true;
        }
        if (gamepad1.dpad_right && !rightLast) {
            selectedStart = selectedStart == StartPos.CLOSE ? StartPos.FAR : StartPos.CLOSE;
            changed = true;
        }

        upLast = gamepad1.dpad_up;   downLast = gamepad1.dpad_down;
        leftLast = gamepad1.dpad_left; rightLast = gamepad1.dpad_right;

        if (changed) {
            super.reset();
            follower.setStartingPose(startPose());
            schedule(buildSequence());
        }

        telemetryM.addData("Alliance",  isBlue ? "BLUE" : "RED");
        telemetryM.addData("Start",     selectedStart);
        telemetryM.addData("Sequence",  selectedSequence);
        telemetryM.addLine("↑↓ sequence  |  ← alliance  |  → start side");
        telemetryM.update();
    }

    // ─── run() ────────────────────────────────────────────────────────────────
    @Override
    public void run() {
        if (loopTimer == null) loopTimer = new ElapsedTime();

        follower.update();

        telemetryM.addData("Loop ms",   loopTimer.milliseconds());
        telemetryM.addData("Pose",      follower.getPose());
        telemetryM.addData("Flywheel",  robot.flywheel.getVelocity());
        telemetryM.addData("FW Target", robot.flywheel.getTargetVelocity());
        telemetryM.addData("FW Ready",  robot.flywheel.atTarget());

        loopTimer.reset();
        robot.updateLoop(telemetryM);
    }

    // ─── end() ────────────────────────────────────────────────────────────────
    @Override
    public void end() {
        savePose(follower.getPose());
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Sequences
    // ═══════════════════════════════════════════════════════════════════════════

    private SequentialCommandGroup buildSequence() {
        switch (selectedSequence) {
            default:        return buildClose18();
        }
    }

    // ─── CLOSE 18 ─────────────────────────────────────────────────────────────
    private SequentialCommandGroup buildClose18() {
        return new SequentialCommandGroup(
            // Set starting position
            setStart(START_CLOSE),
            // Shoot preloads into goal while moving
            shootWhileMoving(CLOSE_SCORE, 3, 1850),
            // Intake PGP Spike Mark
            intakePath(new Pose[]{CLOSE_PGP, CLOSE_PGP_1}, 1),
            // Drive to scoring position and shoot
            moveAndShootClose(3, 1850),
            // Drive and intake from gate
            intakePath(new Pose[]{CLOSE_GATE, CLOSE_GATE_1}, 1),
            wait(1000.0),
            // Drive to scoring position and shoot
            moveAndShootClose(3, 1850),
            // Drive and intake from gate
            intakePath(new Pose[]{CLOSE_GATE, CLOSE_GATE_1}, 1),
            wait(1000.0),
            // Drive to scoring position and shoot
            moveAndShootClose(3, 1850),
            // Intake PPG Spike Mark
            intakePath(new Pose[]{CLOSE_PPG, CLOSE_PPG_1}, 1),
            // Drive to scoring position and shoot
            moveAndShootClose(3, 1850),
            // Intake PPG Spike Mark
            intakePath(new Pose[]{CLOSE_GPP, CLOSE_GPP_1}, 1),
            // Drive to scoring position and shoot
            windUpAndDrive(CLOSE_TOEND, 1850, WindUpAndDrive.HeadingMode.TANGENTIAL_REV, 1),
            moveAndShoot(CLOSE_END, 3, 1850)
        );
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Helper methods
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Drive to targetPose while spinning the flywheel up to flywheelVel.
     * Conveyor stays closed — no balls fired.
     */
    private Command windUpAndDrive(Pose targetPose, double flywheelVel, WindUpAndDrive.HeadingMode headingMode, double driveSpeed) {
        return new WindUpAndDrive(follower, p(targetPose), flywheelVel, headingMode, driveSpeed)
                .withTimeout(3000);
    }
    private Command wait(double milli) {
        return new Wait().withTimeout((long) milli);
    }
    /**
     * Drive to targetPose while firing ballsToFire balls mid-path.
     * Flywheel should already be warm from a preceding windUpAndDrive.
     */
    private SequentialCommandGroup moveAndShootClose(int ballsToFire, double flywheelVel) {
        return new SequentialCommandGroup(
                windUpAndDrive(CLOSE_TOSCORE, 1850, WindUpAndDrive.HeadingMode.TANGENTIAL_REV, 1),
                moveAndShoot(CLOSE_SCORE, 3, 1850)
        );
    }

    private Command shootWhileMoving(Pose targetPose, int balls, double flywheelVel) {
        return new ShootWhileMoving(follower, p(targetPose), balls, flywheelVel, isBlue)
                .withTimeout(3500);
    }

    private Command moveAndShoot(Pose targetPose, int balls, double flywheelVel) {
        return new MoveAndShoot(follower, p(targetPose), balls, flywheelVel, isBlue)
                .withTimeout(3500);
    }
    /**
     * Drive to midPose at full speed, slow to collectSpeed for the final leg
     * to collectPose with intake running, then stop intake.
     */
    private SequentialCommandGroup intakePath(Pose[] collectPoses, double collectSpeed) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.flywheel.off()), // flywheel off while collecting
                new ParallelCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD),
                        new DriveToPose(follower, p(collectPoses), DriveToPose.HeadingMode.TANGENTIAL, collectSpeed).withTimeout(1500)
                ),
                new SetIntake(Intake.MotorState.STOP)
        );
    }

    /** Sets the follower starting pose (must be first in every sequence). */
    private InstantCommand setStart(Pose pose) {
        return new InstantCommand(() -> follower.setStartingPose(p(pose)));
    }

    // ─── Pose + alliance helpers ──────────────────────────────────────────────

    /** Mirror pose for Red alliance if needed. */
    private Pose p(Pose pose) {
        return Poses.forAlliance(pose, isBlue);
    }
    private Pose[] p(Pose[] pose) {
        return Poses.forAlliance(pose, isBlue);
    }

    private Pose startPose() {
        return p(selectedStart == StartPos.CLOSE ? START_CLOSE : START_FAR);
    }

    private void savePose(Pose pose) {
        try (FileWriter writer = new FileWriter("/sdcard/FIRST/pose.txt")) {
            writer.write(pose.getX() + "," + pose.getY() + "," + pose.getHeading());
        } catch (IOException e) {
            telemetry.addLine("WARNING: Failed to save end pose");
        }
    }
}