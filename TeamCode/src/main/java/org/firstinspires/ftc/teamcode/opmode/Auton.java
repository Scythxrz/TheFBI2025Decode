package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.globals.Constants.*;
import static org.firstinspires.ftc.teamcode.config.globals.Poses.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.commandbase.commands.DriveToBlobs;
import org.firstinspires.ftc.teamcode.config.commandbase.commands.DriveToPose;
import org.firstinspires.ftc.teamcode.config.commandbase.commands.MoveAndShoot;
import org.firstinspires.ftc.teamcode.config.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.config.commandbase.commands.ShootWhileMoving;
import org.firstinspires.ftc.teamcode.config.commandbase.commands.PiecewiseHeading;
import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.globals.Poses;
import org.firstinspires.ftc.teamcode.config.globals.Robot;
import org.firstinspires.ftc.teamcode.config.globals.RobotDrawing;
import org.firstinspires.ftc.teamcode.config.globals.PedroConstants;

import java.io.FileWriter;
import java.io.IOException;

import static org.firstinspires.ftc.teamcode.config.commandbase.commands.Shoot.FiringMode.*;
import static org.firstinspires.ftc.teamcode.config.commandbase.commands.DriveToPose.HeadingMode.*;
import static org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Intake.*;

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
    private enum Sequence { ALLIANCE, NO_ALLIANCE, FAR, FAR_GPP }

    private StartPos selectedStart    = StartPos.CLOSE;
    private Sequence selectedSequence = Sequence.NO_ALLIANCE;
    private boolean  isBlue           = true;
    private JoinedTelemetry telemetryM; // initialized in initialize() — telemetry is null at field-init time
    private boolean upLast, downLast, leftLast, rightLast;

    // ─── initialize() ─────────────────────────────────────────────────────────
    @Override
    public void initialize() {
        super.reset();
        OP_MODE_TYPE = OpModeType.AUTO;
        telemetryM = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        robot.init(hardwareMap);
        follower = PedroConstants.createFollower(hardwareMap);
        RobotDrawing.init();
        // Sequence is scheduled in run() on first tick — not here —
        // so the flywheel doesn't spin up during init_loop
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
            sequenceScheduled = false; // re-schedule on next run() tick with updated selection
            follower.setStartingPose(p(startPose()));
        }

        telemetryM.addData("Alliance",  isBlue ? "BLUE" : "RED");
        telemetryM.addData("Start",     selectedStart);
        telemetryM.addData("Sequence",  selectedSequence);
        telemetryM.addLine("↑↓ sequence  |  ← alliance  |  → start side");
        telemetryM.update();
    }
    // ─── run() ────────────────────────────────────────────────────────────────
    private boolean sequenceScheduled = false;

    @Override
    public void run() {
        if (!sequenceScheduled) {
            sequenceScheduled = true;
            follower.setPose(startPose());
            schedule(buildSequence());
        }

        if (loopTimer == null) loopTimer = new ElapsedTime();

        follower.update();
        RobotDrawing.drawDebug(follower);

        telemetryM.addData("Loop ms",   loopTimer.milliseconds());
        telemetryM.addData("Pose",      follower.getPose());
        telemetryM.addData("Flywheel",  robot.flywheel.getVelocity());
        telemetryM.addData("FW Target", robot.flywheel.getTargetVelocity());
        telemetryM.addData("FW Ready",  robot.flywheel.atTarget());
        telemetryM.addData("Start Pose", startPose());

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
            case ALLIANCE:        return buildAlliance();
            case NO_ALLIANCE:     return buildNoAlliance();
            default:              return null;
        }
    }

    // ─── CLOSE 18 w/ Alliance─────────────────────────────────────────────────────────────
    private SequentialCommandGroup buildAlliance() {
        PiecewiseHeading piecewiseScore = new PiecewiseHeading()
                .reversedTangent(0.0, 0.6)                                                    // follow path direction for first 60%
                .facingAwayFromPoint(0.6, 1.0, p(GOAL_BLUE).getX(), p(GOAL_BLUE).getY());  // back of robot faces goal for last 40%
        return new SequentialCommandGroup(
                // Shoot preloads into goal while moving
                swm(CLOSE_SCORE, 3, 1850, LINEAR),
                // Pick up PGP Spike Mark
                intake(new Pose[]{CLOSE_PGP, CLOSE_PGP_1}),
                // Score 3
                shoot(CLOSE_SCORE, 3, 1850, piecewiseScore),
                // Gate intake
                gate(),
                // Score 3
                shoot(CLOSE_SCORE, 3, 1850, piecewiseScore),
                // Gate intake
                gate(),
                // Score 3
                shoot(CLOSE_SCORE, 3, 1850, piecewiseScore),
                // Gate intake
                gate(),
                // Score 3
                shoot(CLOSE_SCORE, 3, 1850, piecewiseScore),
                // Pick up PPG Spike Mark
                intake(new Pose[]{CLOSE_PPG, CLOSE_PPG_1}),
                // Score 3
                shoot(CLOSE_END, 3, 1850, piecewiseScore)
        );
    }

    // ─── CLOSE 18 w/ Alliance─────────────────────────────────────────────────────────────
    private SequentialCommandGroup buildNoAlliance() {
        PiecewiseHeading piecewiseScore = new PiecewiseHeading()
                .reversedTangent(0.0, 0.6)                                                    // follow path direction for first 60%
                .facingAwayFromPoint(0.6, 1.0, p(GOAL_BLUE).getX(), p(GOAL_BLUE).getY());  // back of robot faces goal for last 40%
        return new SequentialCommandGroup(
                // Shoot preloads into goal while moving
                shoot(CLOSE_SCORE, 3, 1850, piecewiseScore),
                // Pick up PGP Spike Mark
                intake(new Pose[]{CLOSE_PGP, CLOSE_PGP_1}),
                // Score 3
                shoot(CLOSE_SCORE, 3, 1850, piecewiseScore),
                // Gate intake
                gate(),
                // Score 3
                shoot(CLOSE_SCORE, 3, 1850, piecewiseScore),
                // Gate intake
                gate(),
                // Score 3
                shoot(CLOSE_SCORE, 3, 1850, piecewiseScore),
                // Pick up PPG Spike Mark
                intake(new Pose[]{CLOSE_PPG, CLOSE_PPG_1}),
                // Score 3
                shoot(CLOSE_SCORE, 3, 1850, piecewiseScore),
                // Pick up GPP Spike Mark
                intake(new Pose[]{CLOSE_GPP, CLOSE_GPP_1}),
                // Score 3
                shoot(CLOSE_END, 3, 1850, piecewiseScore)
        );
    }

    private SequentialCommandGroup buildFarGPP() {
        PiecewiseHeading piecewiseScore = new PiecewiseHeading()
                .reversedTangent(0.0, 0.6)                                                    // follow path direction for first 60%
                .facingAwayFromPoint(0.6, 1.0, p(GOAL_BLUE).getX(), p(GOAL_BLUE).getY());  // back of robot faces goal for last 40%
        return new SequentialCommandGroup(
                // Shoot preloads
                new MoveAndShoot(follower, p(FAR_SCORE), 3, 2400, isBlue, PACED, LINEAR),
                // Pick up GPP Spike Mark
                intake(new Pose[]{CLOSE_GPP, CLOSE_GPP_1}),
                // Score 3
                shoot(CLOSE_END, 3, 2400, piecewiseScore),
                // Blob detect
                blob(),
                // Score 3
                shoot(CLOSE_END, 3, 2400, piecewiseScore),
                // Blob detect
                blob(),
                // Score 3
                shoot(CLOSE_END, 3, 2400, piecewiseScore),
                // Blob detect
                blob(),
                // Score 3
                shoot(CLOSE_END, 3, 2400, piecewiseScore)
        );
    }

    private SequentialCommandGroup buildFar() {
        PiecewiseHeading piecewiseScore = new PiecewiseHeading()
                .reversedTangent(0.0, 0.6)                                                    // follow path direction for first 60%
                .facingAwayFromPoint(0.6, 1.0, p(GOAL_BLUE).getX(), p(GOAL_BLUE).getY());  // back of robot faces goal for last 40%
        return new SequentialCommandGroup(
                // Shoot preloads
                new MoveAndShoot(follower, p(FAR_SCORE), 3, 2400, isBlue, PACED, LINEAR),
                // Blob detect
                blob(),
                // Score 3
                shoot(CLOSE_END, 3, 2400, piecewiseScore),
                // Blob detect
                blob(),
                // Score 3
                shoot(CLOSE_END, 3, 2400, piecewiseScore),
                // Blob detect
                blob(),
                // Score 3
                shoot(CLOSE_END, 3, 2400, piecewiseScore),
                // Blob detect
                blob(),
                // Score 3
                shoot(CLOSE_END, 3, 2400, piecewiseScore)
        );
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Helper methods
    // ═══════════════════════════════════════════════════════════════════════════

    private ShootWhileMoving swm(Pose to, int balls, double vel, DriveToPose.HeadingMode hm) {
        return new ShootWhileMoving(follower, p(to), balls, vel, isBlue, PACED, hm);
    }
    private MoveAndShoot shoot(Pose to, int balls, double vel, PiecewiseHeading hm) {
        return new MoveAndShoot(follower, p(to), balls, vel, isBlue, RAPID, hm);
    }
    private MoveAndShoot shoot(Pose to, int balls, double vel, DriveToPose.HeadingMode hm) {
        return new MoveAndShoot(follower, p(to), balls, vel, isBlue, RAPID, hm);
    }
    private WaitCommand wait(double s) {
        return new WaitCommand((long) s);
    }
    private SequentialCommandGroup intake(Pose[] to) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.flywheel.off()),
                new ParallelCommandGroup(
                        new SetIntake(MotorState.FORWARD),
                        new InstantCommand(() -> robot.conveyor.forward()),
                        new DriveToPose(follower, p(to), TANGENTIAL, 1)
                ),
                new SetIntake(MotorState.STOP)
        );
    }
    private SequentialCommandGroup blob() {
        return new SequentialCommandGroup(
                new DriveToPose(follower, p(FAR_VISION), LINEAR),
                new ParallelCommandGroup(
                        new DriveToBlobs(follower, isBlue, p(FAR_HP)),
                    new SetIntake(Intake.MotorState.FORWARD),
                    new InstantCommand(() -> robot.conveyor.forward())
                )
        );
    }
    private SequentialCommandGroup gate() {
        PiecewiseHeading toGate = new PiecewiseHeading()
            .tangent(0.0, 0.6)
            .linear(0.6, 1.0, p(CLOSE_GATE_1).getHeading(), p(CLOSE_GATE).getHeading());
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.flywheel.off()),
                new ParallelCommandGroup(
                        new SetIntake(Intake.MotorState.FORWARD),
                        new InstantCommand(() -> robot.conveyor.forward()),
                        new DriveToPose(follower, p(new Pose[]{CLOSE_GATE, CLOSE_GATE_1}), toGate, 1.0)
                ),
                new WaitCommand(1000),
                new SetIntake(Intake.MotorState.STOP)
        );
    }
    // ─── Pose + alliance helpers ──────────────────────────────────────────────

    /** Mirror pose for Red alliance if needed. */
    private Pose p(Pose pose) {
        return Poses.forAlliance(pose, isBlue);
    }
    private Pose[] p(Pose[] pose) {
        return Poses.forAlliance(pose, isBlue);
    }
    private double mirror(double deg) {
        double rad = Math.toRadians(deg);
        return isBlue ? rad : Math.PI - rad;
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