package org.firstinspires.ftc.teamcode.config.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.config.globals.Robot;

import java.util.ArrayList;
import java.util.List;

/**
 * LimelightBallDetector — reads color blobs from the Limelight, projects them
 * onto the field, groups them into clusters, and returns the best one to drive to.
 *
 * ── Setup ─────────────────────────────────────────────────────────────────────
 *   1. On the Limelight web dashboard, create a Color pipeline (pipeline index 1
 *      is recommended — pipeline 0 is reserved for AprilTag localization).
 *   2. Use the color thresholder to isolate the ball color (sample/tune in hue).
 *      Enable "Color Blob Detection" mode.
 *   3. Set MIN_BLOB_AREA to filter out noise, and CLUSTER_RADIUS_INCHES to
 *      control how tightly blobs must be grouped to count as one cluster.
 *
 * ── Camera mount ──────────────────────────────────────────────────────────────
 *   CAMERA_HEIGHT_INCHES  — height of the lens above the floor
 *   CAMERA_PITCH_DEGREES  — tilt from horizontal (positive = tilted down)
 *   CAMERA_FORWARD_OFFSET — how far forward the camera is from the robot center
 *   CAMERA_LATERAL_OFFSET — how far left/right the camera is from the robot center
 *   These must be measured on your actual robot and set in Constants.
 *
 * ── Coordinate projection ─────────────────────────────────────────────────────
 *   The Limelight reports blob center as tx/ty (horizontal/vertical degrees from
 *   crosshair). We unproject these angles through the camera mount geometry to
 *   get a ground-plane intersection, then rotate into field coordinates using
 *   the robot's current heading from the Pedro follower.
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *   LimelightBallDetector detector = new LimelightBallDetector(follower);
 *   detector.switchToBallPipeline();          // call once in initialize()
 *   BallCluster best = detector.getBestCluster();  // call each loop tick
 *   if (best != null) { ... drive to best.fieldX, best.fieldY ... }
 *   detector.switchToAprilTagPipeline();      // restore when done
 */
public class LimelightBallDetector {

    // ── Pipeline indices ──────────────────────────────────────────────────────
    public static final int PIPELINE_APRILTAG  = 0;
    public static final int PIPELINE_BALL_COLOR = 1;

    // ── Camera mount geometry — TUNE THESE FOR YOUR ROBOT ────────────────────
    // Height of camera lens above the floor (inches)
    public static double CAMERA_HEIGHT_INCHES   = 10.0;
    // Downward tilt of the camera from horizontal (degrees, positive = looking down)
    public static double CAMERA_PITCH_DEGREES   = 25.0;
    // How far forward the camera is from the robot's Pedro tracking center (inches)
    public static double CAMERA_FORWARD_OFFSET  = 6.0;
    // How far left (+) or right (-) of center the camera is (inches)
    public static double CAMERA_LATERAL_OFFSET  = 0.0;

    // ── Clustering / filtering ────────────────────────────────────────────────
    // Minimum blob area (fraction of frame, 0–100) — filters noise/reflections
    public static double MIN_BLOB_AREA          = 0.3;
    // Two blobs within this field distance (inches) are merged into one cluster
    public static double CLUSTER_RADIUS_INCHES  = 18.0;

    // ─────────────────────────────────────────────────────────────────────────

    private final Follower follower;
    private final Robot    robot = Robot.getInstance();

    public LimelightBallDetector(Follower follower) {
        this.follower = follower;
    }

    // ─── Pipeline control ─────────────────────────────────────────────────────

    public void switchToBallPipeline() {
        robot.limelight.pipelineSwitch(PIPELINE_BALL_COLOR);
    }

    public void switchToAprilTagPipeline() {
        robot.limelight.pipelineSwitch(PIPELINE_APRILTAG);
    }

    // ─── Main API ─────────────────────────────────────────────────────────────

    /**
     * Returns the cluster with the most balls visible in the current frame,
     * or null if no valid blobs are detected.
     *
     * Call each loop tick while in ball-detection mode.
     */
    public BallCluster getBestCluster() {
        List<BallCluster> clusters = getClusters();
        if (clusters.isEmpty()) return null;

        BallCluster best = clusters.get(0);
        for (BallCluster c : clusters) {
            // Primary sort: ball count. Tiebreak: confidence (total blob area).
            if (c.ballCount > best.ballCount ||
                    (c.ballCount == best.ballCount && c.confidence > best.confidence)) {
                best = c;
            }
        }
        return best;
    }

    /**
     * Returns all detected clusters this frame, sorted by ball count descending.
     * Useful for telemetry or when you want to evaluate multiple candidates.
     */
    public List<BallCluster> getClusters() {
        LLResult result = robot.limelight.getLatestResult();
        if (result == null || !result.isValid()) return new ArrayList<>();

        List<LLResultTypes.ColorResult> blobs = result.getColorResults();
        if (blobs == null || blobs.isEmpty()) return new ArrayList<>();

        // Project each valid blob to a field position
        List<double[]> fieldPoints = new ArrayList<>(); // [fieldX, fieldY, area]
        Pose robotPose = follower.getPose();

        for (LLResultTypes.ColorResult blob : blobs) {
            if (blob.getTargetArea() < MIN_BLOB_AREA) continue;

            double[] fieldPos = projectToField(
                    blob.getTargetXDegrees(),
                    blob.getTargetYDegrees(),
                    robotPose
            );
            if (fieldPos == null) continue;

            fieldPoints.add(new double[]{fieldPos[0], fieldPos[1], blob.getTargetArea()});
        }

        if (fieldPoints.isEmpty()) return new ArrayList<>();

        // Merge nearby points into clusters
        return mergeToClusters(fieldPoints);
    }

    // ─── Geometry ─────────────────────────────────────────────────────────────

    /**
     * Projects a Limelight (tx, ty) blob center to a field (x, y) position.
     *
     * Steps:
     *   1. Combine camera pitch with ty to get the total downward angle to the blob.
     *   2. Use the camera height to compute ground distance along the camera's look-ray.
     *   3. Decompose into forward/lateral components in the camera frame.
     *   4. Add camera mount offset to get robot-relative vector.
     *   5. Rotate by robot heading to get field-relative vector.
     *   6. Add robot field position.
     *
     * Returns null if the blob is above the horizon (can't intersect the floor).
     */
    private double[] projectToField(double txDeg, double tyDeg, Pose robotPose) {
        double pitchRad = Math.toRadians(CAMERA_PITCH_DEGREES);
        double tyRad    = Math.toRadians(tyDeg);   // positive = above crosshair
        double txRad    = Math.toRadians(txDeg);   // positive = right of crosshair

        // Total downward angle from horizontal to the blob
        double totalDownAngle = pitchRad - tyRad;

        // Blob must be below the horizon to hit the floor
        if (totalDownAngle <= 0) return null;

        // Ground distance from directly below the camera to the blob (inches)
        double groundDist = CAMERA_HEIGHT_INCHES / Math.tan(totalDownAngle);

        // Lateral offset at that distance (right = positive in camera frame)
        double lateralDist = groundDist * Math.tan(txRad);

        // Add camera mount offsets → now robot-relative (forward = robot heading)
        double robotRelForward  = groundDist  + CAMERA_FORWARD_OFFSET;
        double robotRelLateral  = lateralDist + CAMERA_LATERAL_OFFSET;

        // Rotate into field coordinates using robot heading
        // Pedro: x = right, y = up on the field diagram; heading = 0 points along +y
        double heading = robotPose.getHeading();
        double fieldX  = robotPose.getX()
                + robotRelForward  * Math.sin(heading)
                + robotRelLateral  * Math.cos(heading);
        double fieldY  = robotPose.getY()
                + robotRelForward  * Math.cos(heading)
                - robotRelLateral  * Math.sin(heading);

        return new double[]{fieldX, fieldY};
    }

    /**
     * Merges a list of [fieldX, fieldY, area] points into BallClusters.
     * Two points within CLUSTER_RADIUS_INCHES are merged into the same cluster.
     * The cluster center is the area-weighted centroid of its member blobs.
     */
    private List<BallCluster> mergeToClusters(List<double[]> points) {
        // Each entry: [centerX, centerY, totalArea, count]
        List<double[]> clusters = new ArrayList<>();

        for (double[] pt : points) {
            double px = pt[0], py = pt[1], area = pt[2];
            boolean merged = false;

            for (double[] cl : clusters) {
                double dx   = px - cl[0];
                double dy   = py - cl[1];
                double dist = Math.hypot(dx, dy);

                if (dist <= CLUSTER_RADIUS_INCHES) {
                    // Weighted centroid update
                    double totalArea = cl[2] + area;
                    cl[0] = (cl[0] * cl[2] + px * area) / totalArea;
                    cl[1] = (cl[1] * cl[2] + py * area) / totalArea;
                    cl[2] = totalArea;
                    cl[3] += 1;
                    merged = true;
                    break;
                }
            }

            if (!merged) {
                clusters.add(new double[]{px, py, area, 1});
            }
        }

        List<BallCluster> result = new ArrayList<>();
        for (double[] cl : clusters) {
            result.add(new BallCluster(cl[0], cl[1], (int) cl[3], cl[2]));
        }
        return result;
    }
}