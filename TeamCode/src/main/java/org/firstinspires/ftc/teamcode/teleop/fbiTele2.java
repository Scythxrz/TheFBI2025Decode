package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.functions.Drawing;
import org.firstinspires.ftc.teamcode.functions.BallCounter;
import org.firstinspires.ftc.teamcode.functions.Intake;
import org.firstinspires.ftc.teamcode.functions.Limelight;
import org.firstinspires.ftc.teamcode.functions.Shooter;
import org.firstinspires.ftc.teamcode.teleop.macros.MacroBuilder;
import org.firstinspires.ftc.teamcode.teleop.macros.MacroRunner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Objects;

@Config
@TeleOp
public class fbiTele2 extends OpMode {

    private Follower follower;
    private MacroRunner activeMacro = null;
    private boolean automatedDrive;
    private int selectedOption = 0;
    private final String[] options = {"Red", "Blue"};
    private boolean upPressedLast = false;
    private boolean downPressedLast = false;
    // --- Heading settle logic ---
    private long headingAlignedSince = -1;
    private boolean aimLocked = false;
    private boolean feeding = false;
    boolean waitingForRecovery = false;
    long lastShotTime = 0;

    private double power = 0.5;
    private double dx = 0, dy = 0;
    private double[] shootPose = {53.95941530900375, 85.48144609472519, 314.1272869639285}, shootFarPose = {55.78071467999224, 14.52857702232174, 296.34474304417836}, parkPose = {41.678120770792326, 29.632155050442915, 270}, loadingPose1 = {29.883957681051726, 8.528420075092573, 180}, loadingPose2 = {10.190637412460038, 8.528420075092573, 180}, gatePose1 = {23.24448532090227, 52.95828316329453, 150.9234037943176}, gatePose2 = {14.76948103249472, 57.70973044735917, 151.16438391379378}, goalPose = {2, 142}, resetRed = {12, 8, 270}, resetBlue = {32, 135, 270};

    private int invert = -1;
    private final PIDFController controller = new PIDFController(new PIDFCoefficients(1 , 0, 0, 0.025));
    private boolean headingLock = false;
    private int allianceTagId = 20;
    private double powerOffset = 0;
    private double error;

    private double[] mirrorArray(double[] array) {
        if (array == null) return null;
        double mirroredX = 144 - array[0];
        double mirroredHeading = -array[2];
        return new double[] {mirroredX, array[1], mirroredHeading};
    }

    private Pose toPose(double[] array) {
        return new Pose(array[0], array[1], Math.toRadians(array[2]));
    }

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Intake.INSTANCE.initialize(hardwareMap);
        Shooter.INSTANCE.initialize(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap);

    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_down && !downPressedLast) {
            selectedOption = (selectedOption + 1) % options.length;
        }
        if (gamepad1.dpad_up && !upPressedLast) {
            selectedOption = (selectedOption - 1 + options.length) % options.length;
        }
        upPressedLast = gamepad1.dpad_up;
        downPressedLast = gamepad1.dpad_down;

        telemetry.addLine("=== MENU ===");
        for (int i = 0; i < options.length; i++) {
            if (i == selectedOption) telemetry.addData("> ", options[i]);
            else telemetry.addData("  ", options[i]);
        }
        telemetry.addLine("Press DPAD up/down to select, press Start when ready");
        telemetry.update();
    }
    @Override
    public void start() {
        boolean isBlue = Objects.equals(options[selectedOption], "Blue");
        resetRed = new double[]{132, 8, 90};
        if (!isBlue) {
            shootPose = mirrorArray(shootPose);
            shootFarPose = mirrorArray(shootFarPose);
            parkPose = mirrorArray(parkPose);
            loadingPose1 = mirrorArray(loadingPose1);
            loadingPose2 = mirrorArray(loadingPose2);
            gatePose1 = mirrorArray(gatePose1);
            gatePose2 = mirrorArray(gatePose2);
            resetBlue = mirrorArray(resetBlue);
            resetRed = new double[]{12, 8, 90};
            goalPose = new double[]{142, 142};
            allianceTagId = 24; // DECODE Tag ID
            invert = 1;
        }
        follower = Constants.createFollower(hardwareMap);
        Pose startPose;
        try (BufferedReader reader = new BufferedReader(new FileReader("/sdcard/FIRST/pose.txt"))) {
            String[] parts = reader.readLine().split(",");
            double x = Double.parseDouble(parts[0]);
            double y = Double.parseDouble(parts[1]);
            double h = Double.parseDouble(parts[2]);
            startPose = new Pose(x, y, h);
        } catch (Exception e) {
            startPose = new Pose(0, 0, 0); // Default start pose
        }
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        if (follower == null) return;
        follower.update();
        Drawing.drawDebug(follower);

        dx = goalPose[0] - follower.getPose().getX();
        dy = goalPose[1] - follower.getPose().getY();
        double dis = Math.hypot(dx, dy);

        //power = calculateShooterPower(dis);
        power += powerOffset;

        if (activeMacro != null && activeMacro.isBusy() &&
                (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1)) {

            activeMacro.cancel();
            activeMacro = null;
            automatedDrive = false;
            follower.startTeleopDrive(); // Immediately give back control
        }
    // NOW, handle the main logic
        if (activeMacro != null && activeMacro.isBusy()) {
            activeMacro.update();
        }
    // Only allow manual drive if a macro is NOT running
        else if (!automatedDrive) {
            handleDrive(dis);
        }

    // This block correctly restores control AFTER a macro finishes normally
        if (automatedDrive && (activeMacro == null || !activeMacro.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        handleGamepad1();
        handleGamepad2();
        updateTelemetry(dis);
    }
    private void handleDrive(double distance) {
        double turnPower = 0;
        error = Math.atan2(dy, dx) + Math.PI - follower.getPose().getHeading();
        error = Math.atan2(Math.sin(error), Math.cos(error));
        if (headingLock) {
            double denom = dx * dx + dy * dy;
            double headingVelFF = 0;
            double kV = -0.5;
            if (denom > 1e-6) {
                headingVelFF = (dx * follower.getVelocity().getYComponent() - dy * follower.getVelocity().getXComponent()) / denom;
            }
            controller.updateError(error);
            turnPower = controller.run() + kV * headingVelFF;


            //turnPower = controller.run();
        } else {
            turnPower = -gamepad1.right_stick_x * 0.75;
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * invert,
                -gamepad1.left_stick_x * invert,
                turnPower,
                false
        );
    }
    private void handleGamepad1() {
        if (gamepad1.leftBumperWasPressed()) {
            headingLock = !headingLock;
        }
        /*if (gamepad1.aWasPressed()) {
            activeMacro = new MacroBuilder(follower)
                    .addPath(toPose(shootPose))
                    .build();
            activeMacro.start();
            automatedDrive = true;
        }
        if (gamepad1.xWasPressed()) {
            activeMacro = new MacroBuilder(follower)
                    .setGoalPose(new Pose(goalPose[0], goalPose[1], 0))
                    .addPath(toPose(loadingPose1))
                    .addIntakePath(1, toPose(loadingPose2))
                    .addWait(200)
                    .addShootingPath(power, toPose(shootFarPose))
                    .addShootingAction(power, 3000L)
                    .build();
            activeMacro.start();
            automatedDrive = true;
        }*/
        if (gamepad1.bWasPressed()) {
            activeMacro = new MacroBuilder(follower)
                    .addPath(toPose(parkPose))
                    .build();
            activeMacro.start();
            automatedDrive = true;
        }
        if (gamepad1.yWasPressed()) {
            follower.setPose(toPose(resetRed));
        }
        if (gamepad1.rightBumperWasPressed()) {
            follower.setPose(toPose(resetBlue));    
        }
        /*if (gamepad1.dpadUpWasPressed()) {
            resetRed[1] += 1;
        }
        if (gamepad1.dpadDownWasPressed()) {
            resetRed[1] -= 1;
        }
        if (gamepad1.dpadRightWasPressed()) {
            resetRed[0] += 1;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            resetRed[0] -= 1;
        }*/
    }
    private void handleGamepad2() {
        double dist = Math.hypot(dx, dy);
        long now = System.currentTimeMillis();

        // ===============================
        // Heading stability + aim latch
        // ===============================
        double angularVel = follower.getAngularVelocity(); // rad/s

        boolean headingAlignedInstant =
                Math.abs(error) < 0.1 &&        // aim tolerance
                        Math.abs(angularVel) < 0.3;     // not rotating too fast

        boolean headingSettled = false;

        if (!aimLocked) {
            if (headingAlignedInstant) {
                if (headingAlignedSince < 0) {
                    headingAlignedSince = now;
                }
                if (now - headingAlignedSince > 120) { // ms
                    headingSettled = true;
                    aimLocked = true; // 🔒 latch aim
                }
            } else {
                headingAlignedSince = -1;
            }
        } else {
            headingSettled = true; // once locked, allow rapid fire
        }

        // ===============================
// Shooter logic
// ===============================
        boolean rapidFire = gamepad2.y;
        boolean pacedFire = gamepad2.a;

        if (rapidFire || pacedFire) {

            Shooter.INSTANCE.startShootingTableVel(
                    follower.getVelocity(), dx, dy, dist
            );

            boolean shooterRecovered =
                    Shooter.INSTANCE.getTVelocity() - Shooter.INSTANCE.getVelocity() <= 60;

            // -------------------------------
            // RAPID FIRE (Y) — unchanged
            // -------------------------------
            if (rapidFire) {

                if (!feeding) {
                    if (shooterRecovered && headingSettled) {
                        feeding = true;
                        Shooter.INSTANCE.servosOn(0.9);
                    } else {
                        Shooter.INSTANCE.servosOff();
                    }
                } else {
                    Shooter.INSTANCE.servosOn(1);
                }

            }

            // -------------------------------
            // PACED FIRE (A) — waits between shots
            // -------------------------------
            if (pacedFire && headingSettled) {

                if (!feeding && shooterRecovered && !waitingForRecovery) {
                    // Fire ONE shot
                    feeding = true;
                    waitingForRecovery = true;
                    lastShotTime = now;
                    Shooter.INSTANCE.servosOn(1);
                }

                // Stop feeder shortly after ball passes
                if (feeding && now - lastShotTime > 500) {
                    feeding = false;
                    Shooter.INSTANCE.servosOff();
                }

                // Wait until flywheel recovers before allowing next shot
                if (waitingForRecovery && shooterRecovered) {
                    waitingForRecovery = false;
                }
            }

        } else {
            // Fully stop when driver releases
            Shooter.INSTANCE.servosOff();
            Shooter.INSTANCE.stopShooting();

            feeding = false;
            waitingForRecovery = false;
            aimLocked = false;
            headingAlignedSince = -1;
        }

        // ===============================
        // Intake controls
        // ===============================
        if (gamepad2.x && !gamepad2.y && !gamepad2.a) {
            Intake.INSTANCE.startIntake(1);
        } else if (!gamepad2.y && !gamepad2.a) {
            Intake.INSTANCE.stopIntake();
        }

        if (gamepad2.b) {
            Intake.INSTANCE.reverseIntake();
        }


        // ===============================
        // Safety: reset lock if aim is lost
        // ===============================
        if (Math.abs(error) > 0.25) {
            aimLocked = false;
            headingAlignedSince = -1;
        }
    }



    private void updateTelemetry(double dis) {
        telemetry.addLine("=== Follower Info ===");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", normalizeDegrees(Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Distance to Goal", dis);
        telemetry.addData("Error", error);

        telemetry.addLine("\n=== Limelight Info ===");
        telemetry.addData("Has Target", Limelight.INSTANCE.hasTarget());
        telemetry.addData("Tag ID", Limelight.INSTANCE.getTid());
        //telemetry.addData("Pose 3D", Limelight.INSTANCE.getRobotPose3D(Math.toRadians(follower.getHeading())));
        telemetry.addData("tx", Limelight.INSTANCE.getTx());
        telemetry.addData("Seeing Tag? ", Limelight.INSTANCE.seesTag(allianceTagId));
        telemetry.addData("Distance via Camera", Limelight.INSTANCE.getDistanceToTag(allianceTagId));

        telemetry.addLine("\n=== Shooter Info ===");
        telemetry.addData("Shooter Power", power);
        telemetry.addData("Velocity", Shooter.INSTANCE.getVelocity());
        telemetry.addData("Target Velocity", Shooter.INSTANCE.getTVelocity());
        telemetry.addData("Power Offset", powerOffset);

        telemetry.update();
    }
}
