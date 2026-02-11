package org.firstinspires.ftc.teamcode.teleop.macros;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.functions.Intake;
import org.firstinspires.ftc.teamcode.functions.Shooter;
import org.firstinspires.ftc.teamcode.teleop.pathing.Obstacle;
import org.firstinspires.ftc.teamcode.teleop.pathing.ObstacleFactory;
import org.firstinspires.ftc.teamcode.teleop.pathing.PathPlanner;

import java.util.ArrayList;
import java.util.List;

public class MacroBuilder {

    private final Follower follower;
    private final List<MacroAction> actions = new ArrayList<>();
    private Pose goalPose = new Pose(2, 142, 0); // Default Blue Goal

    public MacroBuilder(Follower follower) {
        this.follower = follower;
    }

    public MacroBuilder setGoalPose(Pose goalPose) {
        this.goalPose = goalPose;
        return this;
    }

    // --- BUILDER METHODS ---

    /**
     * Move to a target pose using "Smart" pathing (Obstacle Avoidance).
     * The path is generated ONLY when this specific action starts.
     */
    public MacroBuilder addSmartPath(Pose targetPose) {
        actions.add(new MacroAction() {
            private boolean pathStarted = false;

            @Override
            public void start() {
                // Generate path from CURRENT location to TARGET
                List<Obstacle> obstacles = ObstacleFactory.fieldObstacles();
                PathChain chain = PathPlanner.generatePath(follower.getPose(), targetPose, obstacles);

                if (chain != null) {
                    follower.followPath(chain, true); // true = hold end
                    pathStarted = true;
                }
            }

            @Override
            public void update() {
                follower.update();
            }

            @Override
            public boolean isFinished() {
                // It's finished if we started a path and the follower is done
                return pathStarted && !follower.isBusy();
            }
        });
        return this;
    }

    /**
     * Move to a target in a straight line (Standard PedroPathing).
     */
    public MacroBuilder addPath(Pose targetPose) {
        actions.add(new MacroAction() {
            private boolean pathStarted = false;

            @Override
            public void start() {
                Path path = new Path(new BezierLine(follower.getPose(), targetPose));
                path.setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getPose().getHeading());
                follower.followPath(path, true);
                pathStarted = true;
            }

            @Override
            public void update() {
                follower.update();
            }

            @Override
            public boolean isFinished() {
                return pathStarted && !follower.isBusy();
            }
        });
        return this;
    }

    /**
     * Run the Shooter logic.
     */
    public MacroBuilder addShootingAction(double power, long durationMs) {
        actions.add(new MacroAction() {
            private long startTime;

            @Override
            public void start() {
                startTime = System.currentTimeMillis();
                updateShooter();
            }

            @Override
            public void update() {
                //updateShooter();
                double targetVel = Shooter.INSTANCE.getTVelocity();
                if (Math.abs(targetVel - Shooter.INSTANCE.getVelocity()) <= 100) {
                    Shooter.INSTANCE.servosOn(1);
                } else {
                    Shooter.INSTANCE.servosOff();
                }
            }

            private void updateShooter() {
                double dx = goalPose.getX() - follower.getPose().getX();
                double dy = goalPose.getY() - follower.getPose().getY();
                double dist = Math.hypot(dx, dy);
                Shooter.INSTANCE.startShootingTable(dist, 0);
            }

            @Override
            public boolean isFinished() {
                if (System.currentTimeMillis() - startTime > durationMs) {
                    Shooter.INSTANCE.stopShooting();
                    Shooter.INSTANCE.servosOff();
                    return true;
                }
                return false;
            }
        });
        return this;
    }
    public MacroBuilder addShootingPath(double power, Pose targetPose) {
        actions.add(new MacroAction() {

            private boolean pathStarted;

            @Override
            public void start() {
                Path path = new Path(new BezierLine(follower.getPose(), targetPose));
                path.setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getPose().getHeading());
                follower.followPath(path, true);
                pathStarted = true;
                updateShooter();
            }

            @Override
            public void update() {
                follower.update();
                updateShooter();
            }

            private void updateShooter() {
                double dx = goalPose.getX() - follower.getPose().getX();
                double dy = goalPose.getY() - follower.getPose().getY();
                double dist = Math.hypot(dx, dy);
                Shooter.INSTANCE.startShootingTable(dist, 0);
            }

            @Override
            public boolean isFinished() {
                if (pathStarted && !follower.isBusy()) {
                    Shooter.INSTANCE.stopShooting();
                    return true;
                }
                return false;
            }
        });
        return this;
    }

    /**
     * Run the Intake while moving or stationary.
     */
    public MacroBuilder addIntakePath(double power, Pose targetPose) {
        actions.add(new MacroAction() {
            private boolean pathStarted;

            @Override
            public void start() {
                Path path = new Path(new BezierLine(follower.getPose(), targetPose));
                path.setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getPose().getHeading());
                follower.setMaxPower(0.5);
                follower.followPath(path, true);
                pathStarted = true;
                Intake.INSTANCE.startIntake(power);
            }

            @Override
            public void update() {follower.update();}

            @Override
            public boolean isFinished() {
                if (pathStarted && !follower.isBusy()) {
                    follower.setMaxPower(1);
                    Intake.INSTANCE.stopIntake();
                    return true;
                }
                return false;
            }
        });
        return this;
    }

    /**
     * Just wait for a set amount of time.
     */
    public MacroBuilder addWaitUntil(boolean condition) {
        actions.add(new MacroAction() {
            @Override
            public void start() { }

            @Override
            public void update() { }

            @Override
            public boolean isFinished() {
                return condition;
            }
        });
        return this;
    }
    public MacroBuilder addWait(long durationMs) {
        actions.add(new MacroAction() {
            private long startTime;
            @Override
            public void start() { startTime = System.currentTimeMillis(); }
            @Override
            public void update() { }
            @Override
            public boolean isFinished() { return System.currentTimeMillis() - startTime > durationMs; }
        });
        return this;
    }

    /**
     * Builds the runner.
     */
    public MacroRunner build() {
        return new MacroRunner(new ArrayList<>(actions));
    }
}
