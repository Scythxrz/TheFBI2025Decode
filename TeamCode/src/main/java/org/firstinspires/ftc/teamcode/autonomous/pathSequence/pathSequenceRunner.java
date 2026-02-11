package org.firstinspires.ftc.teamcode.autonomous.pathSequence;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.functions.BallCounter;
import org.firstinspires.ftc.teamcode.functions.Intake;
import org.firstinspires.ftc.teamcode.functions.Shooter;

import java.util.ArrayList;
import java.util.List;

public class pathSequenceRunner {

    public enum StepType { PATH, INTAKE_PATH, SHOOTER_PATH, SHOOTER_ACTION, WAIT }

    public static class Step {
        public StepType type;
        public PathChain path; // for PATH, INTAKE_PATH, SHOOTER_PATH
        public double maxPower = 1.0; // optional override for INTAKE_PATH
        // power field is deprecated/unused for SHOOTER actions as we use table now
        public double power = 0.47; 
        public boolean vel = false; // Flag for velocity-based shooting
        public long durationMs = 0; // for time-based SHOOTER_ACTION and WAIT
        public boolean timerStarted = false; // for velocity-checked shooting
        public boolean intakeOn = false; // for WAIT steps

        public Step(PathChain path) { this.type = StepType.PATH; this.path = path; }
        public Step(PathChain path, double maxPower) { this.type = StepType.INTAKE_PATH; this.path = path; this.maxPower = maxPower; }

        // Constructor for time-based shooter action
        public Step(long durationMs, double power) {
            this.type = StepType.SHOOTER_ACTION;
            this.durationMs = durationMs;
            this.power = power;
            this.vel = false; // Explicitly time-based
        }

        // Constructor for velocity-based time shooter action
        public Step(long durationMs, double power, boolean vel) {
            this.type = StepType.SHOOTER_ACTION;
            this.durationMs = durationMs;
            this.power = power;
            this.vel = vel;
        }

        public Step(long durationMs, boolean isWait) {
            this.type = StepType.WAIT;
            this.durationMs = durationMs;
            this.intakeOn = false;
        }

        public Step(long durationMs, boolean isWait, boolean intakeOn) {
            this.type = StepType.WAIT;
            this.durationMs = durationMs;
            this.intakeOn = intakeOn;
        }

        public Step(PathChain path, double power, boolean isShooterPath) {
            this.type = StepType.SHOOTER_PATH;
            this.path = path;
            this.power = power;
        }
    }

    private final Follower follower;
    private final Telemetry telemetry;
    private final BallCounter ballCounter;

    private final List<Step> steps = new ArrayList<>();
    private int current = 0;
    private final Timer actionTimer = new Timer();
    private boolean running = false;
    private boolean stepStarted = false;
    private Pose goalPose = new Pose(2, 142, 0); // Default Blue Goal

    public pathSequenceRunner(Follower follower, Telemetry telemetry, BallCounter ballCounter) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.ballCounter = ballCounter;
    }
    
    public void setGoalPose(Pose goal) {
        this.goalPose = goal;
    }

    public pathSequenceRunner addCollectionSequence(
            //PathChain scoreToStart,
            PathChain startToFinish,
            PathChain finishToScore,
            double moveSpeed,
            long shootDurationMs,
            double shootPower,
            boolean withGate,
            PathChain toGate,
            PathChain throughGate,
            PathChain toStartGate,
            boolean vel
    ) {
        //addPath(scoreToStart);
        addIntakePath(startToFinish, moveSpeed);
        if (withGate) {
            addPath(toStartGate);
            addPath(toGate);
            addPath(throughGate);
        }
        addShooterPath(finishToScore, shootPower);
        if (shootDurationMs > 0) {
            addShooterAction(shootDurationMs, shootPower, vel);
        }
        return this;
    }

    public pathSequenceRunner addPath(PathChain path) { steps.add(new Step(path)); return this; }

    public pathSequenceRunner addPath(Path path) { steps.add(new Step(new PathChain(path))); return this; }

    public pathSequenceRunner addIntakePath(PathChain pathChain, double maxPower) { steps.add(new Step(pathChain, maxPower)); return this; }

    public pathSequenceRunner addShooterPath(PathChain path, double shooterPower) { steps.add(new Step(path, shooterPower, true)); return this; }

    public pathSequenceRunner addShooterAction(long durationMs, double power) { steps.add(new Step(durationMs, power)); return this; }

    public pathSequenceRunner addShooterAction(long durationMs, double power, boolean vel) { steps.add(new Step(durationMs, power, vel)); return this; }

    public void start() { if (!running) { running = true; current = 0; stepStarted = false; } }

    public boolean isFinished() { return running && current >= steps.size(); }

    public pathSequenceRunner addWaitAction(long durationMs) { steps.add(new Step(durationMs, true)); return this; }

    public pathSequenceRunner addWaitAction(long durationMs, boolean intakeOn) { steps.add(new Step(durationMs, true, intakeOn)); return this; }

    private void enableIntake(boolean on, double motorPower) {
        if (on) Intake.INSTANCE.startIntake(motorPower);
        else Intake.INSTANCE.stopIntake();
    }

    public void update() {
        if (!running || current >= steps.size()) return;

        Step s = steps.get(current);
        
        // Calculate distance to goal for shooter
        double dx = goalPose.getX() - follower.getPose().getX();
        double dy = goalPose.getY() - follower.getPose().getY();
        double dist = Math.hypot(dx, dy);

        switch (s.type) {
            case PATH:
                if (!stepStarted) {
                    follower.setMaxPower(1.0);
                    follower.followPath(s.path, true);
                    stepStarted = true;
                    telemetry.addData("PSR", String.format("Starting PATH %d", current));
                }
                if (!follower.isBusy() && stepStarted) {
                    stepStarted = false;
                    current++;
                }
                break;
            case INTAKE_PATH:
                if (!stepStarted) {
                    Shooter.INSTANCE.servosOff();
                    enableIntake(true, 1.0);
                    follower.setMaxPower(s.maxPower);
                    follower.followPath(s.path, true);
                    stepStarted = true;
                    telemetry.addData("PSR", String.format("Starting INTAKE_PATH %d (maxPower=%.2f)", current, s.maxPower));
                }
                if (!follower.isBusy() && stepStarted) {
                    enableIntake(false, 0);
                    follower.setMaxPower(1.0);
                    stepStarted = false;
                    current++;
                }
                break;
            case SHOOTER_ACTION:
                // Continuously update shooter power based on distance]
                if (s.power <= 2000) {
                    Shooter.INSTANCE.startShootingVel(s.power);
                } else {
                    Shooter.INSTANCE.startShootingVelFull(s.power);
                }
                if (!stepStarted) {
                    //if (follower.isBusy()) break;
                    // s.power); // Replaced by startShootingTable above
                    actionTimer.resetTimer();
                    stepStarted = true;
                    s.timerStarted = false;
                }
                if (s.vel) { // Velocity-checked time-based shooting
                    double velDiff = Shooter.INSTANCE.getTVelocity() - Shooter.INSTANCE.getVelocity();
                    // Check if we are at the target speed
                    if (s.power > 2000) {
                        if (velDiff <= 150) {
                            // If we are at speed, turn on servos

                            Shooter.INSTANCE.servosOn(1);

                            // *** FIX: Only start the timer if it hasn't been started yet ***
                            if (!s.timerStarted) {
                                actionTimer.resetTimer(); // Start counting duration ONLY when at speed
                                s.timerStarted = true;
                            }
                        } else {
                            Shooter.INSTANCE.servosOff();
                        }
                    } else {
                        if (velDiff <= 100) {
                            // If we are at speed, turn on servos

                            Shooter.INSTANCE.servosOn(1);

                            // *** FIX: Only start the timer if it hasn't been started yet ***
                            if (!s.timerStarted) {
                                actionTimer.resetTimer(); // Start counting duration ONLY when at speed
                                s.timerStarted = true;
                            }
                        }

                    }

                    // Only move to the next step if the timer has started and the duration has passed
                    if (s.timerStarted && actionTimer.getElapsedTime() >= s.durationMs) {
                        Shooter.INSTANCE.stopShooting();
                        Shooter.INSTANCE.servosOff();
                        stepStarted = false;
                        s.timerStarted = false;
                        current++;
                    }
                } else { // Simple time-based shooting (no velocity check)
                    if (actionTimer.getElapsedTime() >= 300) { // Wait a bit for spin-up
                        Shooter.INSTANCE.servosOn(1);
                    }

                    if (actionTimer.getElapsedTime() >= s.durationMs) {
                        Shooter.INSTANCE.stopShooting();
                        Shooter.INSTANCE.servosOff();
                        stepStarted = false;
                        current++;
                    }
                }
                break;
            case SHOOTER_PATH:
                // Continuously update shooter power based on distance
                Shooter.INSTANCE.startShootingVel(1850);

                if (!stepStarted) {
                    //Shooter.INSTANCE.startShooting((s.power) * 0.6); // Replaced by startShootingTable above
                    follower.setMaxPower(1.0);
                    follower.followPath(s.path, true);
                    stepStarted = true;
                    telemetry.addData("PSR", String.format("Starting SHOOTER_PATH %d (power=%.2f)", current, s.power));
                }
                if (!follower.isBusy() && stepStarted) {
                    stepStarted = false;
                    current++;
                }
                break;
            case WAIT:
                if (!stepStarted) {
                    if (follower.isBusy()) break;
                    if (s.intakeOn) enableIntake(true, 1.0); // Turn intake on if requested
                    actionTimer.resetTimer();
                    stepStarted = true;
                    telemetry.addData("PSR", String.format("Starting WAIT %d (ms=%d)", current, s.durationMs));
                }
                if (actionTimer.getElapsedTime() >= s.durationMs && stepStarted) {
                    if (s.intakeOn) enableIntake(false, 0); // Turn intake off if it was turned on for this wait
                    stepStarted = false;
                    current++;
                }
                break;
        }
    }
}
