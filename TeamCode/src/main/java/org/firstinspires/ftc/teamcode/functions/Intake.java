package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Lightweight Intake helper using only FTC SDK.
 * Provides simple imperative methods for turning intake on/off.
 */
public class Intake {

    public static final Intake INSTANCE = new Intake();

    private boolean initialized = false;

    private DcMotorEx intakeMotor, conveyor;
    private CRServo kicker;
    private Servo stopper;

    // Variables for ball detection logic
    private double prevVel = -1.0;
    private double currentVel;

    private Intake() {}

    /** Initialize hardware (must be called once from your OpMode.init()) */
    public void initialize(HardwareMap hardwareMap) {
        if (initialized) return;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        kicker = hardwareMap.get(CRServo.class, "kicker");
        stopper = hardwareMap.get(Servo.class, "stopper");
        conveyor = hardwareMap.get(DcMotorEx.class, "conv");


        // Set motor direction and behavior
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        initialized = true;
    }

    /** Start intake motor and servos. Motor power is -1..1. */
    public void startIntake(double motorPower) {
        intakeMotor.setPower(-motorPower);
        kicker.setPower(-1);
        conveyor.setPower(1);
        stopper.setPosition(0);
    }

    /** Stop intake motor and servos. */
    public void stopIntake() {
        intakeMotor.setPower(0);
        kicker.setPower(0);
        conveyor.setPower(0);
        stopper.setPosition(0);
    }

    public void reverseIntake() {
        intakeMotor.setPower(0.5);
        conveyor.setPower(-0.5);
        stopper.setPosition(0);
    }
    public Double getVelocity() {
        return intakeMotor.getVelocity();
    }
    /**
     * Detects if a ball has passed through the intake.
     * @return 1 if a ball is detected, 0 otherwise.
     */
    public int ballDetection() {
        currentVel = intakeMotor.getVelocity();

        // If prevVel hasn't been set yet, initialize it and exit for this loop cycle.
        if (prevVel == -1.0) {
            prevVel = currentVel;
            return 0; // No detection on the first run
        }

        // A significant drop in velocity indicates a ball was intaked.
        // The threshold of 500 is a starting point and may need tuning for the intake motor.
        if ((prevVel - currentVel) > 600) {
            prevVel = currentVel;
            return 1; // Ball detected
        } else {
            prevVel = currentVel;
            return 0; // No ball detected
        }
    }
}
