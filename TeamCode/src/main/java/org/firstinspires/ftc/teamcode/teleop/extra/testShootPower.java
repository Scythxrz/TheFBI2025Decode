package org.firstinspires.ftc.teamcode.teleop.extra;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.functions.Intake;
import org.firstinspires.ftc.teamcode.functions.Limelight;
import org.firstinspires.ftc.teamcode.functions.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class testShootPower extends OpMode {
    private Follower follower;
    private Servo stopper;
    double power = 1600;
    double dx, dy, distance;
    private List<String> saveCoordinates = new ArrayList<>();
    double[] goalPose = new double[] {0, 144};
    @Override
    public void init() {
        Intake.INSTANCE.initialize(hardwareMap);
        Shooter.INSTANCE.initialize(hardwareMap);
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
    public void start() {
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        follower.update();

        dx = goalPose[0] - follower.getPose().getX();
        dy = goalPose[1] - follower.getPose().getY();
        distance = Math.hypot(dx, dy);

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        if (gamepad1.right_bumper) {
            Intake.INSTANCE.startIntake(1.0);
        } else {
            Intake.INSTANCE.stopIntake();
        }
        if (gamepad1.left_bumper) {
            Shooter.INSTANCE.startShootingVel(power);
            Shooter.INSTANCE.servosOn(1);
        } else {
            Shooter.INSTANCE.servosOff();
            Shooter.INSTANCE.stopShooting();
        }
        if (gamepad1.dpadUpWasPressed()) {
            power += 25;
        }
        if (gamepad1.dpadDownWasPressed()) {
            power -= 25;
        }
        if (gamepad1.bWasPressed()) {
            saveCoordinates.add(Integer.parseInt(String.valueOf(distance)), String.valueOf(Shooter.INSTANCE.getVelocity()));
        }
        telemetry.addLine("SHOOTER TEST");
        telemetry.addData("Target Vel", power);
        telemetry.addData("Shooter Vel", Shooter.INSTANCE.getVelocity());
        telemetry.addData("Distance", distance);
        telemetry.addLine("=== Saved Distance to Velocity ===");
        for (int i = 0; i < saveCoordinates.size(); i++) {
            telemetry.addData("  ", saveCoordinates.get(i));
        }
        telemetry.addData("Pose:", follower.getPose());
    }
}
