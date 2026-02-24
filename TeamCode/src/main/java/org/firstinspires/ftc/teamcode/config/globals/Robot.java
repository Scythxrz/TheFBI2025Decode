package org.firstinspires.ftc.teamcode.config.globals;

import static org.firstinspires.ftc.teamcode.config.globals.Constants.*;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Conveyor;
import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Gate;
import org.firstinspires.ftc.teamcode.config.commandbase.subsystems.Intake;

import java.util.List;

/**
 * Singleton Robot class — owns all hardware and instantiates all subsystems.
 * <p>
 * Subsystems access hardware via Robot.getInstance() rather than holding
 * their own HardwareMap references, keeping hardware initialization centralized.
 *<p>
 * Usage:
 *   Robot robot = Robot.getInstance();
 *   robot.init(hardwareMap);          // call once in OpMode initialize()
 *   robot.updateLoop(telemetryM);     // call every loop() iteration
 *   robot.getVoltage();               // cached battery voltage for compensation
 */
public class Robot extends com.seattlesolvers.solverslib.command.Robot {

    // ─── Singleton ────────────────────────────────────────────────────────────
    private static final Robot instance = new Robot();
    public static Robot getInstance() { return instance; }
    private Robot() {}

    // ─── Hardware ─────────────────────────────────────────────────────────────

    // Drive motors (Mecanum — names must match PedroConstants driveConstants motor names)
    public MotorEx rf, rr, lf, lr;

    // Shooting mechanism
    public DcMotorEx shooterMotor;       // "shooter"
    public Motor.Encoder shooterEncoder; // piggybacks on shooterMotor

    // Conveyor / feeder
    public MotorEx conveyorMotor;        // "conv"

    // Intake
    public MotorEx intakeMotor;          // "intake"

    // Gate / stopper servo
    public Servo stopperServo;           // "stopper"

    // Vision
    public Limelight3A limelight;        // "limelight"

    // Voltage sensor (for shooter velocity compensation)
    public VoltageSensor voltageSensor;
    private double cachedVoltage = 12.0;
    private ElapsedTime voltageTimer;

    // ─── Subsystems ───────────────────────────────────────────────────────────
    public Conveyor conveyor;
    public Flywheel flywheel;
    public Gate gate;
    public Intake intake;

    // ─── Init ─────────────────────────────────────────────────────────────────

    public void init(HardwareMap hwMap) {
        // Bulk caching for faster loop times
        List<LynxModule> hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Voltage sensor
        voltageSensor = hwMap.voltageSensor.iterator().next();
        voltageTimer = new ElapsedTime();

        // Drive motors (used by Pedro Pathing Follower — names must match driveConstants)
        lf = new MotorEx(hwMap, "lf").setCachingTolerance(0.01);
        lr = new MotorEx(hwMap, "lr").setCachingTolerance(0.01);
        rf = new MotorEx(hwMap, "rf").setCachingTolerance(0.01);
        rr = new MotorEx(hwMap, "rr").setCachingTolerance(0.01);

        lf.setRunMode(Motor.RunMode.RawPower);
        lr.setRunMode(Motor.RunMode.RawPower);
        rf.setRunMode(Motor.RunMode.RawPower);
        rr.setRunMode(Motor.RunMode.RawPower);

        // Shooter motor — uses DcMotorEx directly for velocity/PIDF control
        shooterMotor = hwMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        shooterEncoder = new Motor(hwMap, "shooter").encoder;

        // Conveyor
        conveyorMotor = new MotorEx(hwMap, "conv");
        conveyorMotor.setCachingTolerance(0.0001);
        conveyorMotor.setRunMode(Motor.RunMode.RawPower);

        // Intake
        intakeMotor = new MotorEx(hwMap, "intake");
        intakeMotor.setCachingTolerance(0.0001);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Gate servo — opens to let balls through into the flywheel
        stopperServo = hwMap.get(Servo.class, "stopper");
        stopperServo.setPosition(STOPPER_CLOSED);

        // Limelight — start on pipeline 0 (AprilTag)
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // ── Subsystems (no HardwareMap needed — they use Robot.getInstance()) ──
        // SubsystemBase auto-registers with CommandScheduler on construction.
        conveyor = new Conveyor();
        flywheel = new Flywheel();
        gate     = new Gate();
        intake   = new Intake();
    }

    // ─── Voltage helper ───────────────────────────────────────────────────────

    /** Returns a cached battery voltage (polled at VOLTAGE_SENSOR_POLLING_RATE Hz). */
    public double getVoltage() {
        if (voltageTimer == null || voltageTimer.seconds() > (1.0 / VOLTAGE_SENSOR_POLLING_RATE)) {
            cachedVoltage = voltageSensor.getVoltage();
            if (voltageTimer != null) voltageTimer.reset();
        }
        if (Double.isNaN(cachedVoltage) || cachedVoltage == 0) cachedVoltage = 12.0;
        return cachedVoltage;
    }

    // ─── Main loop update ─────────────────────────────────────────────────────

    /**
     * Call once per loop() iteration.
     * Runs the CommandScheduler (which calls periodic() on every registered subsystem)
     * and updates telemetry.
     */
    public void updateLoop(TelemetryManager telemetryData) {
        CommandScheduler.getInstance().run();
        if (telemetryData != null) telemetryData.update();
    }
}