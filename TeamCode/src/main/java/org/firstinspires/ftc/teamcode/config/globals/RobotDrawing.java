package org.firstinspires.ftc.teamcode.config.globals;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

/**
 * RobotDrawing — draws the robot and path history on the Panels field view.
 *
 * Extracted from Tuning.java's package-private Drawing class so it can be
 * used from Auton and TeleOp without depending on Tuning.
 *
 * ── Setup (call once in initialize()) ────────────────────────────────────────
 *   RobotDrawing.init();
 *
 * ── Per-loop usage ────────────────────────────────────────────────────────────
 *   // Robot position + pose trail + current path target:
 *   RobotDrawing.drawDebug(follower);
 *
 *   // Robot position only (no trail, no path):
 *   RobotDrawing.drawRobot(follower.getPose());
 *   RobotDrawing.sendPacket();
 */
public class RobotDrawing {

    public static final double ROBOT_RADIUS = 9;

    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * Call once in initialize() to configure Panels Field with Pedro coordinate offsets.
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * Draws robot position, pose trail, and current path target point.
     * Call every loop tick. Sends the packet automatically.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(
                    follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(
                    closestPoint.getX(),
                    closestPoint.getY(),
                    follower.getCurrentPath().getHeadingGoal(
                            follower.getCurrentPath().getClosestPointTValue())
            ), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);
        sendPacket();
    }

    /**
     * Draws the robot at the given pose using the default style.
     * Call sendPacket() after to flush to Panels.
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * Draws the robot at the given pose using a custom style.
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null
                || Double.isNaN(pose.getX())
                || Double.isNaN(pose.getY())
                || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2;
        double y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent();
        double y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /** Draws a single Path with the given style. */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();
        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) points[j][i] = 0;
            }
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /** Draws all paths in a PathChain with the given style. */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /** Draws the pose history trail. */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);
        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(
                    poseTracker.getXPositionsArray()[i],
                    poseTracker.getYPositionsArray()[i]);
            panelsField.line(
                    poseTracker.getXPositionsArray()[i + 1],
                    poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /** Flushes the current frame to Panels. Call at the end of each loop. */
    public static void sendPacket() {
        panelsField.update();
    }
}