package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.vision.BallCluster;
import org.firstinspires.ftc.teamcode.config.vision.LimelightBallDetector;

/**
 * DriveToBlobs — scans for ball clusters using the Limelight, then drives
 * to the field position with the most detected balls.
 *
 * ── Scan phase ────────────────────────────────────────────────────────────────
 *   The command first spends SCAN_DURATION_MS collecting frames and accumulating
 *   cluster observations. This smooths out single-frame noise — a cluster that
 *   appears consistently across multiple frames scores higher than one that
 *   flickers. After the scan, it drives to the highest-scoring position.
 *
 *   If no cluster is found during the scan, the command ends immediately
 *   without driving anywhere (fallback: the caller should chain a default path).
 *
 * ── Path ──────────────────────────────────────────────────────────────────────
 *   Once a target is chosen, DriveToBlobs builds a straight-line DriveToPose
 *   with TANGENTIAL heading so the intake faces the cluster on arrival.
 *   Always wrap with .withTimeout() in your sequence.
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *   // In Auton sequence — scan then drive, 3 second timeout:
 *   new DriveToBlobs(follower, isBlue).withTimeout(3000)
 *
 *   // With a custom scan duration:
 *   new DriveToBlobs(follower, isBlue, 400)
 */
public class DriveToBlobs extends CommandBase {

    // How many ms to spend accumulating frames before committing to a target
    public static long SCAN_DURATION_MS = 300;

    // Minimum balls in best cluster to bother driving — ignore noise
    public static int MIN_BALLS_TO_PURSUE = 1;

    private enum State { SCANNING, DRIVING, DONE }
    private State state;

    private final Follower              follower;
    private final LimelightBallDetector detector;
    private final long                  scanDuration;
    private final Pose                  fallbackPose; // null = just end if nothing found

    // Accumulated cluster votes: each frame, the best cluster increments its vote
    // We track the running weighted centroid + vote count
    private double accumX, accumY, accumWeight;
    private long   scanStart;
    private int    totalVotes;

    private DriveToPose driveToPose;

    // ─── Constructors ─────────────────────────────────────────────────────────

    /** No fallback — ends immediately if no balls detected. */
    public DriveToBlobs(Follower follower, boolean isBlue) {
        this(follower, isBlue, null, SCAN_DURATION_MS);
    }

    /** With fallback pose — drives there if no balls detected. */
    public DriveToBlobs(Follower follower, boolean isBlue, Pose fallbackPose) {
        this(follower, isBlue, fallbackPose, SCAN_DURATION_MS);
    }

    /** Full constructor — custom scan duration, optional fallback. */
    public DriveToBlobs(Follower follower, boolean isBlue, Pose fallbackPose, long scanDurationMs) {
        this.follower     = follower;
        this.detector     = new LimelightBallDetector(follower);
        this.scanDuration = scanDurationMs;
        this.fallbackPose = fallbackPose;
    }

    // ─── Lifecycle ────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        state        = State.SCANNING;
        scanStart    = System.currentTimeMillis();
        accumX       = 0;
        accumY       = 0;
        accumWeight  = 0;
        totalVotes   = 0;
        driveToPose  = null;

        detector.switchToBallPipeline();
    }

    @Override
    public void execute() {
        switch (state) {

            case SCANNING:
                // Accumulate best-cluster observations each tick
                BallCluster best = detector.getBestCluster();
                if (best != null && best.ballCount >= MIN_BALLS_TO_PURSUE) {
                    double weight = best.ballCount * best.confidence;
                    accumX      += best.fieldX * weight;
                    accumY      += best.fieldY * weight;
                    accumWeight += weight;
                    totalVotes++;
                }

                if (System.currentTimeMillis() - scanStart >= scanDuration) {
                    commitTarget();
                }
                break;

            case DRIVING:
                driveToPose.execute();
                if (driveToPose.isFinished()) {
                    state = State.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        detector.switchToAprilTagPipeline();
        if (driveToPose != null) driveToPose.end(interrupted);
    }

    // ─── Helpers ─────────────────────────────────────────────────────────────

    /**
     * Finalizes the scan and starts driving, or goes directly to DONE
     * if no valid cluster was accumulated.
     */
    private void commitTarget() {
        if (accumWeight < 1e-6 || totalVotes == 0) {
            // Nothing detected — drive to fallback if provided, otherwise just end
            if (fallbackPose != null) {
                driveToPose = new DriveToPose(follower, fallbackPose, DriveToPose.HeadingMode.LINEAR);
                driveToPose.initialize();
                state = State.DRIVING;
            } else {
                state = State.DONE;
            }
            return;
        }

        double targetX = accumX / accumWeight;
        double targetY = accumY / accumWeight;
        Pose   target  = new Pose(targetX, targetY, follower.getPose().getHeading());

        // Drive with tangential heading so the intake faces the cluster on arrival
        driveToPose = new DriveToPose(follower, target, DriveToPose.HeadingMode.TANGENTIAL);
        driveToPose.initialize();
        state = State.DRIVING;
    }
}