package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.globals.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.config.commandbase.commands.LaunchSequence;
import org.firstinspires.ftc.teamcode.config.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.globals.Robot;

import static org.firstinspires.ftc.teamcode.config.globals.PedroConstants.createFollower;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Objects;

/**
 * Main TeleOp — command-based port of FBI2025's fbiTele2.java.
 *
 * Key differences from the original:
 *   • Extends CommandOpMode instead of OpMode
 *   • Hardware lives in Robot singleton, subsystems extend SubsystemBase
 *   • Button bindings are declared once in initialize() via GamepadEx
 *   • The drive + aim loop is still in run() (Pedro Pathing requires per-loop updates)
 *   • Shooting is handled by LaunchSequence (whileActiveContinuous on Y/A)
 *   • Intake is handled by SetIntake instant commands
 */
@Config
@TeleOp(name = "TeleOp", group = "AAATeleOp")
public class Teleop extends CommandOpMode {

    // ─── Gamepad wrappers ─────────────────────────────────────────────────────
    private GamepadEx driver;
    private GamepadEx operator;

    // ─── Pedro Pathing ────────────────────────────────────────────────────────
    private Follower follower;

    // ─── Robot singleton ──────────────────────────────────────────────────────
    private final Robot robot = Robot.getInstance();

    // ─── Telemetry ────────────────────────────────────────────────────────────
    private TelemetryData telemetryData;

    // ─── Aim / heading state (replicated from fbiTele2.handleGamepad2) ────────
    private int     invert            = -1; // -1 = Blue, 1 = Red
    private double  headingError      = 0;
    private double  distanceToGoal    = 0;
    private boolean headingLock       = false;
    private double[] goalPose         = GOAL_POSE_BLUE;

    // Heading PID for aimbot (same gains as FBI2025)
    private final PIDFController headingController =
            new PIDFController(new com.pedropathing.control.PIDFCoefficients(1, 0, 0, 0.025));

    // Reset poses (field coordinates, Pedro system)
    public static double[] RESET_BLUE = {32, 135, 270};
    public static double[] RESET_RED  = {12,   8,  90};

    // Park pose
    public static double[] PARK_POSE = {41.678, 29.632, 270};

    // ─── Alliance selection (init_loop menu) ──────────────────────────────────
    private final String[] options = {"Blue", "Red"};
    private int  selectedOption    = 0;
    private boolean upLast = false, downLast = false;

    // ─── initialize() — called once when the OpMode is initialized ────────────

    @Override
    public void initialize() {
        // Wipe the scheduler from any previous run
        super.reset();

        OP_MODE_TYPE = OpModeType.TELEOP;

        // Telemetry with FTC Dashboard support
        telemetryData = new TelemetryData(
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
        );

        // Initialize all hardware and register subsystems with CommandScheduler
        robot.init(hardwareMap);

        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // ── Driver button bindings ──────────────────────────────────────────

        // Toggle heading lock (aim toward goal)
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> headingLock = !headingLock)
        );

        // Auto-park — drives to park pose via Pedro heading + position lock
        // Replace with a proper Pedro path if you have one tuned
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> {
                    if (follower != null) follower.setPose(toPose(PARK_POSE));
                })
        );

        // Reset pose — Blue side
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> follower.setPose(toPose(RESET_BLUE)))
        );

        // Reset pose — Red side
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> follower.setPose(toPose(RESET_RED)))
        );

        // ── Operator button bindings ────────────────────────────────────────

        // Y — hold to fire (rapid-fire mode: feeds whenever flywheel + heading ready)
        // A — hold to fire (paced mode, same command, LaunchSequence handles pacing internally)
        operator.getGamepadButton(GamepadKeys.Button.Y).whileActiveContinuous(
                new LaunchSequence(() -> distanceToGoal, () -> headingError)
        );

        operator.getGamepadButton(GamepadKeys.Button.A).whileActiveContinuous(
                new LaunchSequence(() -> distanceToGoal, () -> headingError)
        );

        // X — start intake
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SetIntake(Intake.MotorState.FORWARD)
        );
        operator.getGamepadButton(GamepadKeys.Button.X).whenReleased(
                new SetIntake(Intake.MotorState.STOP)
        );

        // B — reverse intake
        operator.getGamepadButton(GamepadKeys.Button.B).whileActiveContinuous(
                new InstantCommand(() -> robot.intake.reverse())
        );
        operator.getGamepadButton(GamepadKeys.Button.B).whenReleased(
                new SetIntake(Intake.MotorState.STOP)
        );
    }

    // ─── initialize_loop() — shows alliance selector while waiting to start ───

    @Override
    public void initialize_loop() {
        if (gamepad1.dpad_down && !downLast) selectedOption = (selectedOption + 1) % options.length;
        if (gamepad1.dpad_up   && !upLast)   selectedOption = (selectedOption - 1 + options.length) % options.length;
        upLast   = gamepad1.dpad_up;
        downLast = gamepad1.dpad_down;

        telemetry.addLine("=== ALLIANCE SELECT ===");
        for (int i = 0; i < options.length; i++) {
            telemetry.addData(i == selectedOption ? "> " : "  ", options[i]);
        }
        telemetry.addLine("DPAD up/down to switch, Start to begin");
        telemetry.update();
    }

    // ─── run() — called repeatedly once the OpMode starts ────────────────────

    private boolean firstRun = true;

    @Override
    public void run() {
        if (firstRun) {
            firstRun = false;

            // Configure alliance-specific values
            boolean isBlue = Objects.equals(options[selectedOption], "Blue");
            if (isBlue) {
                goalPose    = GOAL_POSE_BLUE;
                invert      = -1;
                ALLIANCE_COLOR = AllianceColor.BLUE;
            } else {
                goalPose    = GOAL_POSE_RED;
                invert      = 1;
                ALLIANCE_COLOR = AllianceColor.RED;
                RESET_BLUE  = mirrorArray(RESET_BLUE);
                RESET_RED   = new double[]{132, 8, 90};
            }

            // Build Pedro follower and load starting pose from file
            follower = createFollower(hardwareMap);
            Pose startPose;
            try (BufferedReader reader = new BufferedReader(new FileReader("/sdcard/FIRST/pose.txt"))) {
                String[] parts = reader.readLine().split(",");
                startPose = new Pose(
                        Double.parseDouble(parts[0]),
                        Double.parseDouble(parts[1]),
                        Double.parseDouble(parts[2])
                );
            } catch (Exception e) {
                startPose = new Pose(0, 0, 0);
            }
            follower.setStartingPose(startPose);
            follower.startTeleopDrive();
        }

        // ── Pedro update ──────────────────────────────────────────────────────
        follower.update();

        // ── Compute aim geometry ──────────────────────────────────────────────
        double dx = goalPose[0] - follower.getPose().getX();
        double dy = goalPose[1] - follower.getPose().getY();
        distanceToGoal = Math.hypot(dx, dy);
        headingError   = normalizeAngle(Math.atan2(dy, dx) + Math.PI - follower.getPose().getHeading());

        // ── Drive (only when no path-following command is active) ─────────────
        double turnPower;
        if (headingLock) {
            headingController.updateError(headingError);
            double kV = -0.5;
            double denom = dx * dx + dy * dy;
            double headingVelFF = denom > 1e-6
                    ? (dx * follower.getVelocity().getYComponent() - dy * follower.getVelocity().getXComponent()) / denom
                    : 0;
            turnPower = headingController.run() + kV * headingVelFF;
        } else {
            turnPower = -gamepad1.right_stick_x * 0.75;
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * invert,
                -gamepad1.left_stick_x * invert,
                turnPower,
                false
        );

        // ── Limelight update ──────────────────────────────────────────────────
        // (No subsystem for Limelight — update it directly here each loop)
        // robot.limelight is available if you want to call getLatestResult() etc.

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetryData.addData("X",             follower.getPose().getX());
        telemetryData.addData("Y",             follower.getPose().getY());
        telemetryData.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.addData("Distance",      distanceToGoal);
        telemetryData.addData("Heading Error", headingError);
        telemetryData.addData("Heading Lock",  headingLock);
        telemetryData.addData("Flywheel Vel",  robot.flywheel.getVelocity());
        telemetryData.addData("Flywheel Target", robot.flywheel.getTargetVelocity());
        telemetryData.addData("Flywheel Ready",  robot.flywheel.atTarget());
        telemetryData.addData("Intake State",    Intake.motorState);
        telemetryData.addData("Alliance",        ALLIANCE_COLOR);

        // Runs CommandScheduler (calls periodic() on all subsystems) + flushes telemetry
        robot.updateLoop(telemetryData);
    }

    // ─── end() ────────────────────────────────────────────────────────────────

    @Override
    public void end() {
        robot.intake.stop();
        robot.flywheel.off();
        robot.conveyor.stop();
    }

    // ─── Helpers ──────────────────────────────────────────────────────────────

    private static Pose toPose(double[] arr) {
        return new Pose(arr[0], arr[1], Math.toRadians(arr[2]));
    }

    private static double[] mirrorArray(double[] arr) {
        return new double[]{144 - arr[0], arr[1], -arr[2]};
    }

    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}