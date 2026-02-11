package org.firstinspires.ftc.teamcode.teleop.pathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.teleop.util.Geometry;

import java.util.List;

public class PathPlanner {

    public static PathChain generatePath(
            Pose start,
            Pose target,
            List<Obstacle> obstacles
    ) {

        // 1. Try direct line
        boolean clear = true;
        for (Obstacle o : obstacles) {
            if (Geometry.lineIntersectsObstacle(start, target, o)) {
                clear = false;
                break;
            }
        }

        if (clear) {
            return new PathChain(
                    new Path(new BezierLine(start, target))
            );
        }

        // 2. Try avoidance curves
        BezierCurve best = null;
        double bestLen = Double.POSITIVE_INFINITY;

        for (Obstacle o : obstacles) {
            for (Pose ctrl : avoidancePoints(o, 2.0)) {

                BezierCurve curve =
                        new BezierCurve(start, ctrl, target);

                if (curveClear(curve, obstacles)) {
                    double len = curve.length();
                    if (len < bestLen) {
                        bestLen = len;
                        best = curve;
                    }
                }
            }
        }

        return best != null
                ? new PathChain(new Path(best))
                : new PathChain(new Path(new BezierLine(start, target)));
    }

    private static boolean curveClear(
            BezierCurve curve,
            List<Obstacle> obstacles
    ) {
        for (double t = 0; t <= 1.0; t += 0.05) {
            Pose p = curve.getPose(t);
            for (Obstacle o : obstacles) {
                if (o.contains(p)) return false;
            }
        }
        return true;
    }

    private static List<Pose> avoidancePoints(
            Obstacle o,
            double offset
    ) {
        double midX = (o.xmin + o.xmax) / 2.0;
        double midY = (o.ymin + o.ymax) / 2.0;

        return List.of(
                new Pose(o.xmin - offset, midY),
                new Pose(o.xmax + offset, midY),
                new Pose(midX, o.ymin - offset),
                new Pose(midX, o.ymax + offset)
        );
    }
}
