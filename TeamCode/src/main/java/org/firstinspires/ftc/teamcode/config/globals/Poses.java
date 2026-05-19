package org.firstinspires.ftc.teamcode.config.globals;

import com.pedropathing.geometry.Pose;

/**
 * Central pose library — every field coordinate used in Auto lives here.
 *<p>
 * All poses are defined for BLUE alliance.
 * Call Poses.forAlliance(pose) to automatically mirror for Red.
 *<p>
 * Naming convention:
 *   <SEQUENCE>_<ROLE>_<CONTROL>  e.g. PGP_COLLECT, GATE_PUSH, CLOSE_PGP_1, FAR_SCORE
 *
 */
public class Poses {

    // ─── Goal poses (what the robot aims at) ─────────────────────────────────
    public static final Pose GOAL_BLUE = new Pose(10,   131.5, 0);
    public static final Pose GOAL_RED  = new Pose(134.5, 134.5, 0);

    // ─── Start poses ───────────────────────────────────────────────────────

    public static final Pose START_CLOSE = new Pose(17, 120.7, rad(315));
    public static final Pose START_FAR   = new Pose(48.5,   9.3,  rad(270));

    // ─── Close-side sequences ─────────────────────────────────────────────────

    public static final Pose CLOSE_SCORE = new Pose(56.5, 77, rad(315));
    public static final Pose CLOSE_END   = new Pose(54.853,   102.53, rad(320));
    public static final Pose CLOSE_PGP   = new Pose(13,   64);
    public static final Pose CLOSE_PGP_1 = new Pose(35.292, 55.076);
    public static final Pose CLOSE_GATE = new Pose(8.5, 58.8, rad(153));
    public static final Pose CLOSE_GATE_1 = new Pose(24.811, 47.537, rad(120));
    public static final Pose CLOSE_PPG = new Pose(22, 83.8);
    public static final Pose CLOSE_PPG_1 = new Pose(35.574, 84.051);
    public static final Pose CLOSE_GPP = new Pose(15, 34.887);
    public static final Pose CLOSE_GPP_1 = new Pose(56.381, 31.462);

    // ─── Far-side sequences ───────────────────────────────────────────────────
    public static final Pose FAR_SCORE  = new Pose(61.72,   19,    rad(-66));

    public static final Pose FAR_SPIKE_1 = new Pose(8, 25, rad(-90));
    public static final Pose FAR_SPIKE_2 = new Pose(8, 9.3, rad(-90));

    public static final Pose FAR_GATE = new Pose(8, 30.72);
    public static final Pose FAR_GATE_1 = new Pose(10, 6, 4);

    /** Far-side GPP intake variation */
    public static final Pose FAR_GPP_MID     = new Pose(44.419, 37.3, rad(180));
    public static final Pose FAR_GPP_COLLECT = new Pose(14.93, 36.65, rad(180));


    // ─── Alliance mirroring ───────────────────────────────────────────────────

    /**
     * Returns the pose mirrored for Red alliance if needed.
     * Blue poses are defined with x from the Blue side; Red mirrors across x = 72.
     *<p>
     * Usage:
     *   Pose shoot = Poses.forAlliance(Poses.SCORE_CLOSE, isBlue);
     */
    public static Pose forAlliance(Pose pose, boolean isBlue) {
        return isBlue ? pose : pose.mirror(141.5);
    }
    public static Pose[] forAlliance(Pose[] poses, boolean isBlue) {
        Pose[] result = new Pose[poses.length];
        for (int i = 0; i < poses.length; i++) {
            result[i] = forAlliance(poses[i], isBlue);
        }
        return result;
    }

        /**
         * Returns the correct goal pose for the current alliance.
         */
    public static Pose goal(boolean isBlue) {
        return isBlue ? GOAL_BLUE : GOAL_RED;
    }

    // ─── Private helpers ──────────────────────────────────────────────────────

    /** Converts degrees to radians — keeps the pose declarations readable. */
    private static double rad(double degrees) {
        return Math.toRadians(degrees);
    }
}