package org.firstinspires.ftc.teamcode.teleop.extra;

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
public class fbiTele21person extends OpMode {

    private Follower follower;
    private MacroRunner activeMacro = null;
    private BallCounter ballCounter;
    private boolean automatedDrive;
    private int selectedOption = 0;
    private final String[] options = {"Red", "Blue"};
    private boolean upPressedLast = false;
    private boolean downPressedLast = false;
    private Limelight3A limelight;
    private LLResult lastResult;
    private Pose3D pose3D;
    private double adjDist = 0;
    private double power = 0.5;
    private double dx = 0, dy = 0;
    private double ballCount = 0;
    private double[] shootPose = {53.95941530900375, 85.48144609472519, 314.1272869639285}, shootFarPose = {55.78071467999224, 14.52857702232174, 296.34474304417836}, parkPose = {38.597627301784684, 28.916053228118844, 270}, loadingPose1 = {29.883957681051726, 8.528420075092573, 180}, loadingPose2 = {10.190637412460038, 8.528420075092573, 180}, gatePose1 = {23.24448532090227, 52.95828316329453, 150.9234037943176}, gatePose2 = {14.76948103249472, 57.70973044735917, 151.16438391379378}, goalPose = {2, 142}, resetRed = {12, 8, 270}, resetBlue = {32, 135, 270};

    private final double[][] shooterTable = {

    };
    private int invert = -1;
    private final PIDFController controller = new PIDFController(new PIDFCoefficients(1, 0, 0, 0.025));
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

        Limelight.INSTANCE.update();
        follower.update();
        Drawing.drawDebug(follower);

        boolean isBlue = Objects.equals(options[selectedOption], "Blue");
        boolean limelightHasTarget = Limelight.INSTANCE.seesTag(allianceTagId);
        if (limelightHasTarget) {
            Limelight.INSTANCE.getDistanceToTag(allianceTagId);
            /*
            pose3D = lastResult.getBotpose();
            Position pos = pose3D.getPosition();
            YawPitchRollAngles ori = pose3D.getOrientation();

            double x_field = pos.x;
            double y_field = pos.y;

            x_field *= 39.37;
            y_field *= 39.37;

            double heading_rad = Math.toRadians(ori.getYaw());

            Pose limelightPose = new Pose(x_field, y_field, heading_rad, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);*/
            /*Pose limelightPose = Limelight.INSTANCE.getRobotPose(Math.toDegrees(follower.getHeading()));
            if (limelightPose != null) {
                follower.setPose(limelightPose);
            }*/
        }

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
            handleDrive(limelightHasTarget, dis);
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
    private double calculateShooterPower(double dis) {
        Vector velocity = follower.getVelocity();
        double distanceAlongLine = (dx * velocity.getXComponent() + dy * velocity.getYComponent()) / dis;
        double k = 0.2; // tune this experimentally
        double adjustedDistance = dis - k * distanceAlongLine;
        if (adjustedDistance <= shooterTable[0][0]) return shooterTable[0][1];
        for (int i = 0; i < shooterTable.length - 1; i++) {
            if (adjustedDistance >= shooterTable[i][0] && adjustedDistance <= shooterTable[i + 1][0]) {
                double d1 = shooterTable[i][0], p1 = shooterTable[i][1];
                double d2 = shooterTable[i + 1][0], p2 = shooterTable[i + 1][1];
                return p1 + (adjustedDistance - d1) * (p2 - p1) / (d2 - d1);
            }
        }
        return shooterTable[shooterTable.length - 1][1];
    }
    private void handleDrive(boolean limelightHasTarget, double distance) {
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
        }
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
        if (gamepad1.a || gamepad1.y) {
            //Shooter.INSTANCE.startShootingTable(dist, adjDist);
            Shooter.INSTANCE.startShootingTableVel(follower.getVelocity(), dx, dy, dist);
            if (gamepad1.a) {
                if (Shooter.INSTANCE.getTVelocity() - Shooter.INSTANCE.getVelocity() <= 100 && Math.abs(error) < .3) {
                    Shooter.INSTANCE.servosOn(1);
                } else {
                    Shooter.INSTANCE.servosOff();
                }
            } else {
                if (Shooter.INSTANCE.getTVelocity() - Shooter.INSTANCE.getVelocity() <= 100 && Math.abs(error) < .1) {
                    Shooter.INSTANCE.servosOn(1);
                }
            }
        } else {
            Shooter.INSTANCE.servosOff();
            Shooter.INSTANCE.stopShooting();
        }
        if (gamepad1.right_bumper && !gamepad1.y && !gamepad1.a) {
            Intake.INSTANCE.startIntake(1);
        } else if (!gamepad1.y && !gamepad1.a) {
            Intake.INSTANCE.stopIntake();
        }
        if (gamepad1.b) {
            Intake.INSTANCE.reverseIntake();
        }
        if (gamepad2.dpadUpWasPressed()) {
            adjDist += 12.5;
        }
        if (gamepad2.dpadDownWasPressed()) {
            adjDist -= 12.5;
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
