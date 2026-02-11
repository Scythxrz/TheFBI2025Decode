package org.firstinspires.ftc.teamcode.functions;

import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    public static final Shooter INSTANCE = new Shooter();
    private boolean initialized = false;
    private DcMotorEx shooter;
    private Timer conveyorTimer;
    private boolean feedingStarted = false;
    private DcMotor conveyor, intake;
    private CRServo kicker;
    private Servo stopper;
    private HardwareMap hardwareMap;
    private double targetVelocity = 0;
    public static double MAX_MOTOR_VELOCITY = 2600; // 6000 RPM motor with 28 CPR encoder

    // Initialize to a sentinel value to indicate it's not set yet.
    private double prevVel = -1.0;
    private double currentVel;
    private final double[][] shooterTable = {
            {50.45, 1650},
            {60.2258, 1700},
            {65.14, 1725},
            {70.04, 1750},
            {75.6015, 1750},
            {80.3616, 1840},
            {85.6286, 1875},
            {90.393, 1925},
            {95.1473, 1975},
            {100.556, 2025},
            {105.4738, 2050},
            {111.2026, 2100},
            {114.2744, 2150},
            {120.4631, 2200},
            {124.8829, 2275},
            {130.4457, 2350},
            {134.4726, 2375},
            {141.0453, 2475}
    };
    private Shooter() {
        /*{49.6659, 1600},
        {55.2072, 1600},
        {60.2618, 1625},
        {65.1, 1650}
        {70.0354, 1675}
        */
    }

    public void initialize(HardwareMap hardwareMap) {
        if (initialized) return;
        this.hardwareMap = hardwareMap;
        this.conveyorTimer = new Timer();

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        conveyor = hardwareMap.get(DcMotor.class, "conv");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopper = hardwareMap.get(Servo.class, "stopper");
        kicker = hardwareMap.get(CRServo.class, "kicker");
        PIDFCoefficients pidf = new PIDFCoefficients(40, 0, 0, 11);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        stopper.setPosition(0);
        initialized = true;
    }

    // --- Shooter control API ---//

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
    }
    private double calculateShooterPowerVel(Vector velocity, double dx, double dy, double dis) {
        double distanceAlongLine = (dx * velocity.getXComponent() + dy * velocity.getYComponent()) / dis;
        double k = 0.5; // tune this experimentally
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
    public void startShooting(double power) {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double nominalVoltage = 14.0;
        double velocity = MAX_MOTOR_VELOCITY * power;
        this.targetVelocity = power; //(MAX_MOTOR_VELOCITY * power) * (voltage / nominalVoltage);
        shooter.setVelocity(velocity);
    }
    public void startShootingVel(double velocity) {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double nominalVoltage = 13;
        this.targetVelocity = velocity;
        shooter.setVelocity(velocity * (1.0 + 0.5 * ((nominalVoltage / voltage) - 1.0)));//(nominalVoltage / voltage));
    }
    public void startShootingVelFull(double velocity) {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double nominalVoltage = 13;
        this.targetVelocity = velocity;
        shooter.setVelocity(velocity * (nominalVoltage/voltage));//* (1.0 + 0.5 * ((nominalVoltage / voltage) - 1.0)));//(nominalVoltage / voltage));
    }
    public void startShootingTable(double dis, double offset) {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double nominalVoltage = 14.0;
        this.targetVelocity = calculateShooterPower(dis);
        shooter.setVelocity((this.targetVelocity + offset) * (1.0 + 0.5 * ((nominalVoltage / voltage) - 1.0)));
    }
    public void startShootingTableVel(Vector velocity, double dx, double dy, double dis) {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double nominalVoltage = 14.0;
        this.targetVelocity = calculateShooterPowerVel(velocity, dx, dy, dis);
        shooter.setVelocity(this.targetVelocity); //* (1.0 + 0.5 * ((nominalVoltage / voltage) - 1.0)));
    }
    public double getVelocity() {
        return shooter.getVelocity();
    }
    public double getTVelocity() {
        return this.targetVelocity;
    }
    public double getPrevVel() {
        return prevVel;
    }
    public void stopShooting() {
        shooter.setPower(0);
    }

    public int ballDetection() {
        currentVel = this.getVelocity();
        int ballDetected = 0;
        if (prevVel == -1.0) {
            prevVel = currentVel;
            return 0; // No detection on the first run
        }

        if ((prevVel - currentVel) > 175) {
            ballDetected = 1; // Ball detected
            prevVel = currentVel;
        } else if (currentVel > prevVel){
            prevVel = currentVel; // No ball detected
        }
        return ballDetected;
    }
    public void servosOn(double power) {
        stopper.setPosition(1); // Move stopper first
        intake.setPower(-1);

        if (!feedingStarted) {
            conveyorTimer.resetTimer();
            feedingStarted = true;
        }

        if (conveyorTimer.getElapsedTime() > 250) {
            conveyor.setPower(power);
            kicker.setPower(power);
        }
    }

    public void servosOff() {
        conveyor.setPower(0);
        kicker.setPower(0);
        stopper.setPosition(0);
        feedingStarted = false; // Reset the flag
    }
}
