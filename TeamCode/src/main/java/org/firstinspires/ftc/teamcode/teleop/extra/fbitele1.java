package org.firstinspires.ftc.teamcode.teleop.extra;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.functions.Drawing;
import org.firstinspires.ftc.teamcode.functions.Intake;
import org.firstinspires.ftc.teamcode.functions.Limelight;
import org.firstinspires.ftc.teamcode.functions.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Objects;
import java.util.function.Supplier;

// Note to self the light indicator is in pluged into 4 expansion hub and called lightInd
/* Note to Nathan Tran by Christian B. Please delete after use
    I've added and wired the light indicator to the Expansion hub and plugged it into 4 slot
    I will try to add the light to the code but I don't know how to hardware map it into the code
 */
@Config
//@TeleOp
public class fbitele1 extends OpMode {
    private Follower follower;
    private boolean automatedDrive;
    private Supplier<PathChain> shootPath, parkPath, loadingPath, gatePath;
    private Servo velLight;
    private final double slowModeMultiplier = 1;
    private int selectedOption = 0;
    private final String[] options = {"Red", "Blue"};
    private boolean upPressedLast = false;
    private boolean downPressedLast = false;

    private double power = 0.5;
    private double dx = 0, dy = 0;
    private double ballCount = 0;
    private final double[] bShoot = {-87.5, 56, 45}, bPark = {-33.1, 105.5, 90}, bLoading = {-23.25, 121, 325}, bGate = {-80, 25.25, 90}, bGoal = {-130, 6}, bStart = {-74.25, 60.25, 42};
    private double[] aShoot = {0, 0, 0}, aPark = {0, 0, 0}, aLoading = {0, 0, 0}, aGate = {0, 0, 0}, aGoal = {0, 0};

    private final double[][] shooterTable = {
            {54.1, .45},
            {71.56, .45},
            {83, .45},
            {90.5, .475},
            {100.6, 0.5},
            {125.6, 0.525},
            {145.75, 0.55},
    };
    private final PIDFController controller = new PIDFController(new PIDFCoefficients(-0.5, 0, -0.01, 0));
    private boolean headingLock = false;
    private boolean llheadingLock = false;
    private int allianceTagId;
    // A positive value aims LEFT of the tag, a negative value aims RIGHT.
    private static final double AIM_OFFSET_SIDE_INCHES = 13.47; // 5 inches to the left

    // A positive value aims UP from the tag, a negative value aims DOWN.
    // To aim "backwards" is equivalent to aiming "up" on the angled backboard.
    private static final double AIM_OFFSET_UP_INCHES = 12.41; // 5 inches "backwards" (up)
    private boolean park = false;
    private boolean shooting = false;
    private double powerOffset = 0;
    private double x, y = 0;
    private double[] goalVars;
    private Pose mirrorPose(Pose pose) {
        if (pose == null) return null;
        //double mirroredX = -144 - pose.getX();
        double mirroredY = 144 - pose.getY();
        double mirroredHeading = -pose.getHeading();
        return new Pose(pose.getX(), mirroredY, mirroredHeading);
    }

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        velLight = hardwareMap.get(Servo.class, "velLight");
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
        if (isBlue) {
            aShoot = bShoot;
            aLoading = bLoading;
            aPark = bPark;
            aGate = bGate;
            aGoal = bGoal;
            allianceTagId = 20; // DECODE Tag ID
            goalVars = new double[]{-149, 0};
        } else { // Red
            Pose mirroredShoot = mirrorPose(new Pose(bShoot[0], bShoot[1], Math.toRadians(bShoot[2])));
            aShoot = new double[]{mirroredShoot.getX(), mirroredShoot.getY(), Math.toDegrees(mirroredShoot.getHeading())};

            Pose mirroredPark = mirrorPose(new Pose(bPark[0], bPark[1], Math.toRadians(bPark[2])));
            aPark = new double[]{mirroredPark.getX(), mirroredPark.getY(), Math.toDegrees(mirroredPark.getHeading())};

            Pose mirroredLoading = mirrorPose(new Pose(bLoading[0], bLoading[1], Math.toRadians(bLoading[2])));
            aLoading = new double[]{mirroredLoading.getX(), mirroredLoading.getY(), Math.toDegrees(mirroredLoading.getHeading())};

            Pose mirroredGate = mirrorPose(new Pose(bGate[0], bGate[1], Math.toRadians(bGate[2])));
            aGate = new double[]{mirroredGate.getX(), mirroredGate.getY(), Math.toDegrees(mirroredGate.getHeading())};

            Pose mirroredGoal = mirrorPose(new Pose(bGoal[0], bGoal[1], 0));
            aGoal = new double[]{mirroredGoal.getX(), mirroredGoal.getY()};
            goalVars = new double[]{-149, 145};
            allianceTagId = 24; // DECODE Tag ID
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
        follower.update();

        shootPath = () -> createPathTo(aShoot);
        parkPath = () -> createPathTo(aPark);
        loadingPath = () -> createPathTo(aLoading);
        gatePath = () -> createPathTo(aGate);

        follower.startTeleopDrive();
    }

    private PathChain createPathTo(double[] target) {
        return follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(target[0], target[1], Math.toRadians(target[2])))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(target[2]), 0.8))
                .build();
    }

    @Override
    public void loop() {
        if (follower == null) return;

        Limelight.INSTANCE.update();
        follower.update();
        Drawing.drawDebug(follower);

        boolean isBlue = Objects.equals(options[selectedOption], "Blue");
        boolean limelightHasTarget = Limelight.INSTANCE.seesTag(allianceTagId);
        Drawing.drawDebug(follower);
        if (limelightHasTarget) {
            Limelight.INSTANCE.getDistanceToTag(allianceTagId);
            /*Pose limelightPose = Limelight.INSTANCE.getRobotPose(isBlue);
            if (limelightPose != null) {
                follower.setPose(limelightPose);
            }*/
        }


        dx = aGoal[1] - follower.getPose().getY();
        dy = aGoal[0] - follower.getPose().getX();
        double dis = Math.hypot(dx, dy);

        power = calculateShooterPower(dis);
        power += powerOffset;

        if (park && !follower.isBusy()) {
            follower.holdPoint(new BezierPoint(aPark[0], aPark[1]), aPark[2], true);
        }
        if (!automatedDrive) {
            handleDrive(limelightHasTarget, dis, shooting);
        }

        if (automatedDrive && !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        handleGamepad1();
        handleGamepad2();
        updateTelemetry(dis);
    }

    private double calculateShooterPower(double dis) {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double nominalVoltage = 14.0;
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
    /*
    private double calculateShooterPower(double dis) {
        if (dis <= shooterTable[0][0]) return shooterTable[0][1];
        for (int i = 0; i < shooterTable.length - 1; i++) {
            if (dis >= shooterTable[i][0] && dis <= shooterTable[i + 1][0]) {
                double d1 = shooterTable[i][0], p1 = shooterTable[i][1];
                double d2 = shooterTable[i + 1][0], p2 = shooterTable[i + 1][1];
                return p1 + (dis - d1) * (p2 - p1) / (d2 - d1);
            }
        }
        return shooterTable[shooterTable.length - 1][1];
    }*/

    //happy_birthday_JT

    private void handleDrive(boolean limelightHasTarget, double distance, boolean shooting) {
        double turnPower = 0;
        if (headingLock) {
            double error = Math.atan2(goalVars[1] - follower.getPose().getY(), goalVars[0] - follower.getPose().getX()) + Math.toRadians(180) - follower.getHeading();
            error = Math.atan2(Math.sin(error), Math.cos(error));
            if (limelightHasTarget) {
                Double tx = Limelight.INSTANCE.getTxOfTag(allianceTagId, distance);
                if (tx != null) {
                    error = (error + Math.toRadians(-tx)) / 2;
                }
            }
            //controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
            controller.updateError(error);
            turnPower = controller.run();
        } else if (llheadingLock) {
            //limelightHasTarget = false;
            if (limelightHasTarget) {
                Double tx = Limelight.INSTANCE.getTxOfTag(allianceTagId, distance);
                if (tx != null) {
                    /*//double[] aimOffsets = Limelight.INSTANCE.getAimingAngleOffsets(distance, AIM_OFFSET_SIDE_INCHES, AIM_OFFSET_UP_INCHES);
                    double finalAimErrorDegrees = -tx;
                    if (aimOffsets != null) {
                        double horizontalOffset = aimOffsets[0];
                        // Add the horizontal offset to the error.
                        // A positive offset (left) should make the robot turn more left.
                        finalAimErrorDegrees -= horizontalOffset;
                    }*/
                    controller.updateError(Math.toRadians(-tx));
                    turnPower = controller.run();
                } else {
                    turnPower = 0;
                }
            } else {
                double error = Math.atan2(goalVars[1] - follower.getPose().getY(), goalVars[0] - follower.getPose().getX()) + Math.toRadians(180) - follower.getHeading();
                error = Math.atan2(Math.sin(error), Math.cos(error));
                //controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
                controller.updateError(error);
                turnPower = controller.run();
            }
        } else {
            turnPower = gamepad1.right_stick_x * 0.75;
        }
        if (shooting) {
            follower.setTeleOpDrive(
                    0,
                    0,
                    turnPower,
                    true
            );
        } else {
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y * slowModeMultiplier,
                    gamepad1.left_stick_x * slowModeMultiplier,
                    turnPower,
                    true
            );
        }
    }
    private void handleGamepad1() {
        if (gamepad1.leftBumperWasPressed()) {
            llheadingLock = false;
            headingLock = !headingLock;
        }
        if (gamepad1.rightBumperWasPressed()) {
            headingLock = false;
            llheadingLock = !llheadingLock;
        }
        /*
        if (gamepad1.aWasPressed()) {
            automatedDrive = true;
            follower.followPath(shootPath.get());
        }*/
        if (gamepad1.bWasPressed()) {
            automatedDrive = true;
            follower.followPath(parkPath.get());
            park = true;
        }/*
        if (gamepad1.yWasPressed()) {
            automatedDrive = true;
            follower.followPath(loadingPath.get());
        }
        if (gamepad1.xWasPressed()) {
            automatedDrive = true;
            follower.followPath(gatePath.get());
        }*/
    }

    private void handleGamepad2() {
        if (gamepad2.x) {
            ballCount += Intake.INSTANCE.ballDetection();
            Intake.INSTANCE.startIntake(1.0);
        }
        else if (gamepad2.b) {
            Intake.INSTANCE.startIntake(-1.0);
            Shooter.INSTANCE.servosOn(1);
        }
        else Intake.INSTANCE.stopIntake();
        if (gamepad2.left_bumper) Shooter.INSTANCE.servosOn(-1);
        else if (!gamepad2.a) Shooter.INSTANCE.servosOff();

        if (gamepad2.a || gamepad2.y) {
            Shooter.INSTANCE.startShooting(power);
            ballCount -= Shooter.INSTANCE.ballDetection(); // This line was duplicated, now it's not.

            // Auto-servo logic for 'A' (with Limelight)
            boolean isReadyToShootA = (Math.abs(Shooter.INSTANCE.getVelocity() - Shooter.INSTANCE.getTVelocity()) <= 100)
                    && Limelight.INSTANCE.seesTag(allianceTagId) && headingLock;

            // Auto-servo logic for 'Y' (without Limelight)
            boolean isReadyToShootY = (Math.abs(Shooter.INSTANCE.getVelocity() - Shooter.INSTANCE.getTVelocity()) <= 100);
            if (isReadyToShootY) {
                velLight.setPosition(1);
            } else {
                velLight.setPosition(0);
            }
            if ((gamepad2.a && isReadyToShootA) || (gamepad2.y && isReadyToShootY)) {
                Shooter.INSTANCE.servosOn(-1);
            } else {
                // Only turn servos off if the left bumper isn't also trying to turn them on.
                if (!gamepad2.left_bumper) {
                    Shooter.INSTANCE.servosOff();
                }
            }
        } else {
            Shooter.INSTANCE.stopShooting();
            velLight.setPosition(0);
        }
        if (gamepad2.dpadUpWasPressed()) powerOffset += 0.01;
        if (gamepad2.dpadDownWasPressed()) powerOffset -= 0.01;
        if (gamepad2.dpadLeftWasPressed()) powerOffset = 0;
    }

    private double normalizeDegrees(double degrees) {
        double modified = degrees % 360;
        return (modified + 360) % 360;
    }

    private void updateTelemetry(double dis) {
        telemetry.addLine("=== Follower Info ===");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", normalizeDegrees(Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Distance to Goal", dis);
        telemetry.addData("Heading Lock Error", controller.getError());

        telemetry.addLine("\n=== Limelight Info ===");
        telemetry.addData("Has Target", Limelight.INSTANCE.hasTarget());
        telemetry.addData("Tag ID", Limelight.INSTANCE.getTid());
        telemetry.addData("tx", Limelight.INSTANCE.getTx());
        telemetry.addData("Seeing Tag? ", Limelight.INSTANCE.seesTag(allianceTagId));
        telemetry.addData("Distance via Camera", Limelight.INSTANCE.getDistanceToTag(allianceTagId));

        telemetry.addLine("\n=== Shooter Info ===");
        telemetry.addData("Shooter Power", power);
        telemetry.addData("Velocity", Shooter.INSTANCE.getVelocity());
        telemetry.addData("Target Velocity", Shooter.INSTANCE.getTVelocity());
        telemetry.addData("Balls", ballCount);
        telemetry.addData("Power Offset", powerOffset);

        telemetry.update();
    }
}
//Happy_Birthday_JT