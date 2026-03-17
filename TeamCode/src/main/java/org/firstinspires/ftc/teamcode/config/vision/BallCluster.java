package org.firstinspires.ftc.teamcode.config.vision;

/**
 * BallCluster — a group of nearby detected balls projected onto the field.
 *
 * Created by LimelightBallDetector when it groups raw color blobs
 * into spatial clusters. The field position is in Pedro inches,
 * same coordinate system as all Poses.
 */
public class BallCluster {

    public final double fieldX;     // Pedro field X (inches)
    public final double fieldY;     // Pedro field Y (inches)
    public final int    ballCount;  // number of blobs merged into this cluster
    public final double confidence; // sum of blob areas — higher = more certain

    public BallCluster(double fieldX, double fieldY, int ballCount, double confidence) {
        this.fieldX     = fieldX;
        this.fieldY     = fieldY;
        this.ballCount  = ballCount;
        this.confidence = confidence;
    }

    @Override
    public String toString() {
        return String.format("BallCluster(x=%.1f, y=%.1f, count=%d, conf=%.2f)",
                fieldX, fieldY, ballCount, confidence);
    }
}