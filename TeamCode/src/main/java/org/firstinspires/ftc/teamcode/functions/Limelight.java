package org.firstinspires.ftc.teamcode.functions;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


/**
 * A wrapper class for the official Limelight FTC library.
 * This class provides a singleton interface to the Limelight, handles initialization,
 * and performs the necessary coordinate transformations to work with the rest of our codebase.
 * This version manually parses the JSON result to work around private methods in the library.
 */
public class Limelight {
    public static final Limelight INSTANCE = new Limelight();

    private Limelight3A limelight;
    private IMU imu; // Your IMU object
    private LLResult lastResult;
    private boolean initialized = false;

    double limelightLensHeightInches = 14;

    double limelightMountAngleDegrees = 4;

    double goalHeightInches = 29.5;

    private Limelight() {
    }

    public void initialize(HardwareMap hardwareMap) {
        if (initialized) return;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        initialized = true;
    }

    public Double getDistanceToTag(int id) {
        if (lastResult == null) return null;

        for (LLResultTypes.FiducialResult fid : lastResult.getFiducialResults()) {
            if (fid.getFiducialId() == id) {
                double ty = fid.getTargetYDegrees();

                double angleToGoalDegrees = limelightMountAngleDegrees + ty;
                double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

                if (Math.abs(Math.tan(angleToGoalRadians)) < 1e-6) return null;

                return (goalHeightInches - limelightLensHeightInches)
                        / Math.tan(angleToGoalRadians);
            }
        }

        return null;
    }

    public void update() {
        if (!initialized) return;
        lastResult = limelight.getLatestResult();
    }

    public boolean hasTarget() {
        return lastResult != null &&
                lastResult.getFiducialResults() != null &&
                !lastResult.getFiducialResults().isEmpty();
    }

    public boolean seesTag(int id) {
        if (lastResult == null) return false;

        for (LLResultTypes.FiducialResult fid : lastResult.getFiducialResults()) {
            if (fid.getFiducialId() == id) {
                return true;
            }
        }

        return false;
    }

    public Double getTxOfTag(int id, double distance) {
        if (lastResult == null) return null;
        double cameraOffsetX = Math.toDegrees(Math.atan(1.25 / distance));
        for (LLResultTypes.FiducialResult fid : lastResult.getFiducialResults()) {
            if (fid.getFiducialId() == id) {
                double rawTx = fid.getTargetXDegrees();
                return rawTx - cameraOffsetX;  // FIXED
            }
        }

        return null;
    }

    /**
     * Calculates the horizontal (X) and vertical (Y) angular offsets required to aim
     * at a point offset from the AprilTag.
     *
     * @param distanceToTag The perpendicular distance from the camera to the tag (adjacent side).
     * @param offsetSide    The perpendicular distance from the tag to the aim point, parallel to the backboard's surface (e.g., left/right).
     * @param offsetUp      The perpendicular distance from the tag to the aim point, vertically along the backboard's surface (e.g., up/down).
     * @return A double array containing [angularOffsetX, angularOffsetY] in degrees, or null.
     */
    public double[] getAimingAngleOffsets(double distanceToTag, double offsetSide, double offsetUp) {
        if (distanceToTag <= 0) return null;

        // Calculate the horizontal angle offset (for tx)
        double angularOffsetX = Math.toDegrees(Math.atan(offsetSide / distanceToTag));

        // Calculate the vertical angle offset (for ty)
        double angularOffsetY = Math.toDegrees(Math.atan(offsetUp / distanceToTag));

        return new double[]{angularOffsetX, angularOffsetY};
    }

    public Double getTyOfTag(int id, double distance) {
        if (lastResult == null) return null;
        double cameraOffsetY = Math.toDegrees(Math.atan(2.5 / distance));
        for (LLResultTypes.FiducialResult fid : lastResult.getFiducialResults()) {
            if (fid.getFiducialId() == id) {
                double rawTy = fid.getTargetYDegrees();
                return rawTy - cameraOffsetY;  // FIXED
            }
        }

        return null;
    }

    public int getTid() {
        if (!hasTarget()) return -1;

        if (lastResult.getFiducialResults().isEmpty()) return -1;

        return lastResult.getFiducialResults().get(0).getFiducialId();
    }


    public double getTx() {
        return lastResult != null ? lastResult.getTx() : 0;
    }

    public Pose getRobotPose(double heading) {

        if (!hasTarget()) return null;
        limelight.updateRobotOrientation(heading);
        if (lastResult != null && lastResult.isValid()) {
            Pose3D botpose_mt2 = lastResult.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                return new Pose(x * 39.37, y * 39.37, heading, InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            } else {
                return null;
            }
        }
        return null;
    }
    public Pose3D getRobotPose3D(double heading) {

        if (!hasTarget()) return null;
        limelight.updateRobotOrientation(heading);
        if (lastResult != null && lastResult.isValid()) {
            Pose3D botpose_mt2 = lastResult.getBotpose_MT2();
            if (botpose_mt2 != null) {
                return botpose_mt2;
            } else {
                return null;
            }
        }
        return null;
    }
}
        /*
        Pose3D pose3D = lastResult.getBotpose();
        Position pos = pose3D.getPosition();
        YawPitchRollAngles ori = pose3D.getOrientation();

        double x_field = pos.x;
        double y_field = pos.y;

        x_field *= 39.37;
        y_field *= 39.37;

        x_field += 72;
        y_field += 72;

        double heading_rad = Math.toRadians(ori.getYaw() - 90);

        return new Pose(x_field, y_field, heading_rad, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);*/


//Happy_birthday_JT




