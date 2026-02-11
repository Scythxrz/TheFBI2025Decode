package org.firstinspires.ftc.teamcode.teleop.extra;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.functions.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

@Config
@TeleOp
public class pointFinder extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(32, 135, Math.toRadians(270)); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private List<String> saveCoordinates = new ArrayList<>();
    int selectedOption;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        Drawing.drawDebug(follower);

        follower.setTeleOpDrive(
                gamepad1.left_stick_y * 0.5,
                gamepad1.left_stick_x * 0.5,
                -gamepad1.right_stick_x * 0.5,
                true // Robot Centric
        );
        if (gamepad1.dpadDownWasPressed() && !saveCoordinates.isEmpty()) {
            selectedOption = (selectedOption + 1) % saveCoordinates.size();
        }
        if (gamepad1.dpadUpWasPressed() && !saveCoordinates.isEmpty()) {
            selectedOption = (selectedOption - 1 + saveCoordinates.size()) % saveCoordinates.size();
        }
        if (gamepad1.xWasPressed()) {
            saveCoordinates.remove(selectedOption);
        }
        if (gamepad1.bWasPressed()) {
            saveCoordinates.add(String.valueOf(follower.getPose()));
        }
        telemetry.addLine("=== Saved Coordinates ===");
        for (int i = 0; i < saveCoordinates.size(); i++) {
            if (i == selectedOption) telemetry.addData("> ", saveCoordinates.get(i));
            else telemetry.addData("  ", saveCoordinates.get(i));
        }
        telemetry.addData("Pose:", follower.getPose());
    }
}