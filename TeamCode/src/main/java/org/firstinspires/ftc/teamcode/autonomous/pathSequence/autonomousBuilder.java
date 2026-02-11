package org.firstinspires.ftc.teamcode.autonomous.pathSequence;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.functions.BallCounter;

public class autonomousBuilder {
    private final Follower follower;
    private final Telemetry telemetry;
    private final pathSequenceRunner runner;
    private final boolean isBlue;
    private BallCounter ballCounter;

    private Pose startPose;
    private Pose scorePose;
    private Pose[] startPoses = new Pose[3];
    private Pose[] finishPoses = new Pose[3];
    private Pose endPose;
    private Pose gateStart;
    private Pose gateFinish;
    private Pose lastPose = null;
    private Pose goalPose = new Pose(2, 142, 0); // Default Blue Goal

    public autonomousBuilder(Follower follower, Telemetry telemetry, boolean isBlue, BallCounter ballCounter) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.isBlue = isBlue;
        this.ballCounter = ballCounter;
        this.runner = new pathSequenceRunner(follower, telemetry, this.ballCounter);
    }

    public autonomousBuilder withPoses(
            Pose start, Pose score, Pose ppgStart, Pose ppgFinish,
            Pose pgpStart, Pose pgpFinish, Pose gppStart, Pose gppFinish,
            Pose gateStart, Pose gateFinish, Pose end) {
        if (!isBlue) {
            start = start.mirror();
            score = score.mirror();
            ppgStart = ppgStart.mirror();
            ppgFinish = ppgFinish.mirror();
            pgpStart = pgpStart.mirror();
            pgpFinish = pgpFinish.mirror();
            gppStart = gppStart.mirror();
            gppFinish = gppFinish.mirror();
            gateStart = gateStart.mirror();
            gateFinish = gateFinish.mirror();
            end = end.mirror();
            goalPose = new Pose(142, 142, 0); // Red Goal
        } else {
            goalPose = new Pose(2, 142, 0); // Blue Goal
        }
        this.runner.setGoalPose(goalPose);

        this.startPose = start;
        this.scorePose = score;
        this.startPoses[0] = ppgStart;
        this.finishPoses[0] = ppgFinish;
        this.startPoses[1] = pgpStart;
        this.finishPoses[1] = pgpFinish;
        this.startPoses[2] = gppStart;
        this.finishPoses[2] = gppFinish;
        this.gateStart = gateStart;
        this.gateFinish = gateFinish;
        this.endPose = end;
        follower.setStartingPose(startPose);
        this.lastPose = startPose;
        return this;
    }

    public autonomousBuilder withCoordinateArrays(
            double[] startCoords, double[] scoreCoords, double[] start1Coords, double[] finish1Coords,
            double[] start2Coords, double[] finish2Coords, double[] start3Coords, double[] finish3Coords,
            double[] gateStartCoords, double[] gateEndCoords, double[] endCoords) {
        return withPoses(
                new Pose(startCoords[0], startCoords[1], Math.toRadians(startCoords[2])),
                new Pose(scoreCoords[0], scoreCoords[1], Math.toRadians(scoreCoords[2])),
                new Pose(start1Coords[0], start1Coords[1], Math.toRadians(start1Coords[2])),
                new Pose(finish1Coords[0], finish1Coords[1], Math.toRadians(finish1Coords[2])),
                new Pose(start2Coords[0], start2Coords[1], Math.toRadians(start2Coords[2])),
                new Pose(finish2Coords[0], finish2Coords[1], Math.toRadians(finish2Coords[2])),
                new Pose(start3Coords[0], start3Coords[1], Math.toRadians(start3Coords[2])),
                new Pose(finish3Coords[0], finish3Coords[1], Math.toRadians(finish3Coords[2])),
                new Pose(gateStartCoords[0], gateStartCoords[1], Math.toRadians(gateStartCoords[2])),
                new Pose(gateEndCoords[0], gateEndCoords[1], Math.toRadians(gateEndCoords[2])),
                new Pose(endCoords[0], endCoords[1], Math.toRadians(endCoords[2]))
        );
    }

    // Default to velocity-checked shot
    public autonomousBuilder score() { return score(0.5, 3000L); }
    public autonomousBuilder score(double power) { return score(power, 3000L); }

    // Velocity-checked, time-based shot
    public autonomousBuilder score(double power, long durationMs) {
        /*PathChain toScore = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), scorePose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading()).build();*/
        //runner.addShooterPath(toScore, 1750).addShooterAction(durationMs, power, true);
        runner.addShooterAction(durationMs, power, true);
        this.lastPose = scorePose;
        return this;
    }
    // Simple time-based shot
    public autonomousBuilder scoreTime(double power, long durationMs) {
        PathChain toScore = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), scorePose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading()).build();
        runner.addPath(toScore).addShooterAction(durationMs, power, false);
        this.lastPose = scorePose;
        return this;
    }
    public autonomousBuilder scoreFromStart() { return scoreFromStart(0.47, 3000L, true); }
    public autonomousBuilder scoreFromStart(double power) { return scoreFromStart(power, 3000L, true); }
    public autonomousBuilder scoreFromStart(double power, long durationMs) { return scoreFromStart(power, durationMs, true); }

    public autonomousBuilder scoreFromStart(double power, long durationMs, boolean vel) {
        Path startScore = new Path(new BezierLine(startPose, scorePose));
        startScore.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        runner.addPath(startScore).addShooterAction(durationMs, power, vel);
        this.lastPose = scorePose;
        return this;
    }

    public autonomousBuilder pathPPG(double power, boolean withGate, boolean vel) { return collectFromScore(0, power, 3000L, withGate, vel); }
    public autonomousBuilder pathPGP(double power, boolean withGate, boolean vel) { return collectFromScore(1, power, 3000L, withGate, vel); }
    public autonomousBuilder pathGPP(double power, boolean withGate, boolean vel) { return collectFromScore(2, power, 3000L, withGate, vel); }
    public autonomousBuilder pathPPG(double power, long durationMs, boolean withGate, boolean vel) { return collectFromScore(0, power, durationMs, withGate, vel); }
    public autonomousBuilder pathPGP(double power, long durationMs, boolean withGate, boolean vel) { return collectFromScore(1, power, durationMs, withGate, vel); }
    public autonomousBuilder pathGPP(double power, long durationMs, boolean withGate, boolean vel) { return collectFromScore(2, power, durationMs, withGate, vel); }


    private class setPower implements Runnable {
        @Override
        public void run() {

        }
    }
    private autonomousBuilder collectFromScore(int posIndex, double shootPower, long shootDurationMs, boolean withGate, boolean vel) {
        double moveSpeed = 0.7;
        Pose start = startPoses[posIndex], finish = finishPoses[posIndex];
        /*PathChain toStart = follower.pathBuilder().addPath(new BezierLine(lastPose, start)).setLinearHeadingInterpolation(lastPose.getHeading(), start.getHeading()).build();
        PathChain collect = follower.pathBuilder().addPath(new BezierLine(start, finish)).setLinearHeadingInterpolation(start.getHeading(), finish.getHeading()).build();
        */
        PathChain collect = follower.pathBuilder().addPath(new BezierCurve(lastPose, start, finish)).setTangentHeadingInterpolation().build();
        Pose toScoreStart = withGate ? gateFinish : finish;
        PathChain toScore = follower.pathBuilder().addPath(new BezierLine(toScoreStart, this.scorePose)).setLinearHeadingInterpolation(toScoreStart.getHeading(), scorePose.getHeading()).build();
        PathChain toStartGate = follower.pathBuilder().addPath(new BezierLine(finish, start)).setLinearHeadingInterpolation(finish.getHeading(), start.getHeading()).build();
        PathChain toGate = follower.pathBuilder().addPath(new BezierLine(finish, this.gateStart)).setLinearHeadingInterpolation(finish.getHeading(), this.gateStart.getHeading()).build();
        PathChain throughGate = follower.pathBuilder().addPath(new BezierLine(this.gateStart, this.gateFinish)).setLinearHeadingInterpolation(this.gateStart.getHeading(), this.gateFinish.getHeading()).build();
        runner.addCollectionSequence(/*toStart, */collect, toScore, moveSpeed, shootDurationMs, shootPower, withGate, toGate, throughGate, toStartGate, vel);
        this.lastPose = scorePose;
        return this;
    }

    public autonomousBuilder customPath(Pose from, Pose to, Boolean intake, boolean shooter, double pathSpeed) {
        if (!isBlue) {
            if (from != null) from = from.mirror();
            to = to.mirror();
        }
        Pose effectiveFrom = from != null ? from : (this.lastPose != null ? this.lastPose : follower.getPose());
        if (pathSpeed == 0) pathSpeed = 1;
        PathChain path = follower.pathBuilder().addPath(new BezierLine(effectiveFrom, to)).setLinearHeadingInterpolation(effectiveFrom.getHeading(), to.getHeading()).build();
        if (intake) runner.addIntakePath(path, pathSpeed); else if (shooter) runner.addShooterPath(path, 1); else runner.addPath(path);
        this.lastPose = to;
        return this;
    }

    public autonomousBuilder customPath(Pose to, boolean intake, boolean shooter, double pathSpeed) { return customPath(null, to, intake, shooter, pathSpeed); }

    public autonomousBuilder customPath(Pose[] poses, boolean intake, boolean shooter, double pathSpeed, boolean reversed, boolean tangential) {
        return customPath(null, poses, intake, shooter, pathSpeed, reversed, tangential);
    }

    public autonomousBuilder customPath(Pose from, Pose[] poses, boolean intake, boolean shooter, double pathSpeed, boolean reversed, boolean tangential) {
        Pose effectiveFrom = from;
        Pose[] processedPoses = new Pose[poses.length];

        if (!isBlue) {
            if (effectiveFrom != null) effectiveFrom = effectiveFrom.mirror();
            for (int i = 0; i < poses.length; i++) {
                processedPoses[i] = poses[i].mirror();
            }
        } else {
            System.arraycopy(poses, 0, processedPoses, 0, poses.length);
        }

        if (effectiveFrom == null) effectiveFrom = (this.lastPose != null ? this.lastPose : follower.getPose());
        if (pathSpeed == 0) pathSpeed = 1;

        Pose[] allPoses = new Pose[processedPoses.length + 1];
        allPoses[0] = effectiveFrom;
        System.arraycopy(processedPoses, 0, allPoses, 1, processedPoses.length);

        Path pathObj = new Path(new BezierCurve(allPoses));
        if (tangential) {
            pathObj.setTangentHeadingInterpolation();
            if (reversed) {
                pathObj.reverseHeadingInterpolation();
            }
        } else {
            pathObj.setLinearHeadingInterpolation(effectiveFrom.getHeading(), allPoses[allPoses.length - 1].getHeading());
        }

        PathChain path = follower.pathBuilder().addPath(pathObj).build();
        if (intake) runner.addIntakePath(path, pathSpeed); else if (shooter) runner.addShooterPath(path, 1); else runner.addPath(path);
        this.lastPose = allPoses[allPoses.length - 1];
        return this;
    }

    public autonomousBuilder gatePath(Pose start) {
        PathChain toGate = follower.pathBuilder().addPath(new BezierLine(start, this.gateStart)).setLinearHeadingInterpolation(start.getHeading(), this.gateStart.getHeading()).build();
        PathChain hitGate = follower.pathBuilder().addPath(new BezierLine(this.gateStart, this.gateFinish)).setLinearHeadingInterpolation(this.gateStart.getHeading(), this.gateFinish.getHeading()).build();
        runner.addPath(toGate); runner.addPath(hitGate);
        this.lastPose = this.gateFinish;
        return this;
    }

    public autonomousBuilder gatePath() { return gatePath(lastPose); }

    public autonomousBuilder finish() {
        PathChain park = follower.pathBuilder().addPath(new BezierLine(lastPose, endPose)).setLinearHeadingInterpolation(lastPose.getHeading(), endPose.getHeading()).build();
        runner.addPath(park);
        this.lastPose = endPose;
        return this;
    }

    public autonomousBuilder wait(int milliseconds) { runner.addWaitAction(milliseconds); return this; }
    public autonomousBuilder wait(int milliseconds, boolean intakeOn) { runner.addWaitAction(milliseconds, intakeOn); return this; }
    public void start() { runner.start(); }
    public void update() { runner.update(); }
    public boolean isFinished() { return runner.isFinished(); }
}
// donovans hair is cool