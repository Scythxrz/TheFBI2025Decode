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
    public static final Pose GOAL_BLUE = new Pose(2,   142, 0);
    public static final Pose GOAL_RED  = new Pose(142, 142, 0);

    // ─── Start poses ───────────────────────────────────────────────────────

    public static final Pose START_CLOSE = new Pose(17.6, 118.688, rad(315));
    public static final Pose START_FAR   = new Pose(50.5,   8,  rad(270));

    // ─── Close-side sequences ─────────────────────────────────────────────────

    public static final Pose CLOSE_SCORE = new Pose(56.99, 79, rad(315));
    public static final Pose CLOSE_TOSCORE = new Pose(56.99, 79);
    public static final Pose CLOSE_END   = new Pose(60.102,   97.98, rad(320));
    public static final Pose CLOSE_TOEND = new Pose(60.102,   97.98);
    public static final Pose CLOSE_PGP   = new Pose(12.224,   61.419);
    public static final Pose CLOSE_PGP_1 = new Pose(35.292, 53.076);
    public static final Pose CLOSE_GATE = new Pose(10.771, 59.8);
    public static final Pose CLOSE_GATE_1 = new Pose(24.811, 52.537);
    public static final Pose CLOSE_PPG = new Pose(17.352, 83.8);
    public static final Pose CLOSE_PPG_1 = new Pose(35.574, 84.051);
    public static final Pose CLOSE_GPP = new Pose(13.531, 34.887);
    public static final Pose CLOSE_GPP_1 = new Pose(56.381, 31.462);

    // ─── Far-side sequences ───────────────────────────────────────────────────
    public static final Pose FAR_SCORE  = new Pose(57.5,   15,    rad(290));
    public static final Pose FAR_END    = new Pose(40.5,    8,    rad(270));
    public static final Pose FAR_LOAD_1 = new Pose(15,     20,    rad(210));
    public static final Pose FAR_LOAD_2 = new Pose(15,     10,    rad(210));
    public static final Pose FAR_LOAD_3 = new Pose(35,     15,    rad(215));
    public static final Pose FAR_LOAD_4 = new Pose(12,      9,    rad(180));

    /** Far-side GPP intake variation */
    public static final Pose FAR_GPP_MID     = new Pose(46.802, 32.6, rad(180));
    public static final Pose FAR_GPP_COLLECT = new Pose(23.573, 32.6, rad(180));


    // ─── Alliance mirroring ───────────────────────────────────────────────────

    /**
     * Returns the pose mirrored for Red alliance if needed.
     * Blue poses are defined with x from the Blue side; Red mirrors across x = 72.
     *<p>
     * Usage:
     *   Pose shoot = Poses.forAlliance(Poses.SCORE_CLOSE, isBlue);
     */
    public static Pose forAlliance(Pose pose, boolean isBlue) {
        return isBlue ? pose : pose.mirror();
    }
    public static Pose[] forAlliance(Pose[] pose, boolean isBlue) {
        for(int poses = 0; poses < pose.length; poses++) {
            pose[poses] = forAlliance(pose[poses], isBlue);
        }
        return pose;
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