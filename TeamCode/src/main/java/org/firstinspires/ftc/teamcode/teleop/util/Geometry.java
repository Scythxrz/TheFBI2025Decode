package org.firstinspires.ftc.teamcode.teleop.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.teleop.pathing.Obstacle;

public class Geometry {

    public static boolean lineIntersectsObstacle(
            Pose a,
            Pose b,
            Obstacle o
    ) {
        if (o.contains(a) || o.contains(b)) return true;

        Pose r1 = new Pose(o.xmin, o.ymin);
        Pose r2 = new Pose(o.xmax, o.ymin);
        Pose r3 = new Pose(o.xmax, o.ymax);
        Pose r4 = new Pose(o.xmin, o.ymax);

        return segmentsIntersect(a, b, r1, r2) ||
                segmentsIntersect(a, b, r2, r3) ||
                segmentsIntersect(a, b, r3, r4) ||
                segmentsIntersect(a, b, r4, r1);
    }

    // Standard segment intersection
    public static boolean segmentsIntersect(Pose p1, Pose p2, Pose q1, Pose q2) {
        int o1 = orientation(p1, p2, q1);
        int o2 = orientation(p1, p2, q2);
        int o3 = orientation(q1, q2, p1);
        int o4 = orientation(q1, q2, p2);

        if (o1 != o2 && o3 != o4) return true;

        if (o1 == 0 && onSegment(p1, q1, p2)) return true;
        if (o2 == 0 && onSegment(p1, q2, p2)) return true;
        if (o3 == 0 && onSegment(q1, p1, q2)) return true;
        if (o4 == 0 && onSegment(q1, p2, q2)) return true;

        return false;
    }

    private static int orientation(Pose p, Pose q, Pose r) {
        double val = (q.getY() - p.getY()) * (r.getX() - q.getX()) -
                (q.getX() - p.getX()) * (r.getY() - q.getY());
        if (Math.abs(val) < 1e-6) return 0; // Collinear
        return (val > 0) ? 1 : 2; // Clockwise or Counterclockwise
    }

    private static boolean onSegment(Pose p, Pose q, Pose r) {
        return q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX()) &&
                q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY());
    }
}
