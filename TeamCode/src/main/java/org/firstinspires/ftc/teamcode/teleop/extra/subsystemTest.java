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

@Config
@TeleOp
public class subsystemTest extends OpMode {
    private Follower follower;
    private Servo stopper;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();
        Intake.INSTANCE.initialize(hardwareMap);
        Shooter.INSTANCE.initialize(hardwareMap);
        stopper = hardwareMap.get(Servo.class, "stopper");
    }
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        follower.update();
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
            Shooter.INSTANCE.startShootingVel(2600);
            if (Shooter.INSTANCE.getTVelocity() - Shooter.INSTANCE.getVelocity() <= 100) {
                Shooter.INSTANCE.servosOn(1);
            } else {
                Shooter.INSTANCE.servosOff();
            }
        } else {
            Shooter.INSTANCE.stopShooting();
            Shooter.INSTANCE.servosOff();
        }
        /*if (gamepad1.x) {
            stopper.setPosition(1);
        } else {
            stopper.setPosition(0.2);
        }*/
        telemetry.addLine("SHOOTER TEST");
        telemetry.addData("Shooter Vel", Shooter.INSTANCE.getVelocity());
        telemetry.addData("Shooter Target Vel", Shooter.INSTANCE.getTVelocity());
    }
}
