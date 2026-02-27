package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.paths.HeadingInterpolator;

import java.util.ArrayList;
import java.util.List;

/**
 * PiecewiseHeading — fluent builder for piecewise heading interpolation.
 *<p>
 * Lets you specify different heading behaviors across segments of a single path,
 * identified by t-values from 0.0 (path start) to 1.0 (path end).
 *<p>
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *<p>
 *   PiecewiseHeading heading = new PiecewiseHeading()
 *       .add(0.0, 0.4, HeadingInterpolator.linear(0, Math.toRadians(180)))
 *       .add(0.4, 0.7, HeadingInterpolator.facingPoint(72, 72))
 *       .add(0.7, 1.0, HeadingInterpolator.tangent);
 *<p>
 *   // Then pass to any command:
 *   new DriveToPose(follower, target, heading)
 *   new WindUpAndDrive(follower, target, 1850, heading, 1.0)
 *   new MoveAndShoot(follower, target, 3, 1850, isBlue, FiringMode.RAPID, heading)
 *   new ShootWhileMoving(follower, target, 3, 1850, isBlue, FiringMode.RAPID, heading)
 *<p>
 * ── Segment shortcuts ─────────────────────────────────────────────────────────
 *<p>
 *   .linear(t0, t1, fromRad, toRad)          → HeadingInterpolator.linear(...)
 *   .reversedLinear(t0, t1, fromRad, toRad)  → HeadingInterpolator.reversedLinear(...)
 *   .facingPoint(t0, t1, x, y)               → HeadingInterpolator.facingPoint(...)
 *   .tangent(t0, t1)                          → HeadingInterpolator.tangent
 *   .add(t0, t1, interpolator)               → any HeadingInterpolator directly
 *<p>
 * ── Notes ─────────────────────────────────────────────────────────────────────
 *   Segments must cover 0.0–1.0 with no gaps and no overlaps.
 *   Pedro will throw if the piecewise nodes don't tile cleanly.
 */
public class PiecewiseHeading {

    private final List<HeadingInterpolator.PiecewiseNode> nodes = new ArrayList<>();

    // ─── Core add ─────────────────────────────────────────────────────────────

    /** Add a segment with any HeadingInterpolator. */
    public PiecewiseHeading add(double t0, double t1, HeadingInterpolator interpolator) {
        nodes.add(new HeadingInterpolator.PiecewiseNode(t0, t1, interpolator));
        return this;
    }

    // ─── Shortcuts ────────────────────────────────────────────────────────────

    /** Linearly interpolate heading from fromRad to toRad over [t0, t1]. */
    public PiecewiseHeading linear(double t0, double t1, double fromRad, double toRad) {
        return add(t0, t1, HeadingInterpolator.linear(fromRad, toRad));
    }

    /** Reversed linear heading from fromRad to toRad over [t0, t1]. */
    public PiecewiseHeading reversedLinear(double t0, double t1, double fromRad, double toRad) {
        return add(t0, t1, HeadingInterpolator.reversedLinear(fromRad, toRad));
    }

    /** Robot faces a fixed field point (x, y) over [t0, t1]. */
    public PiecewiseHeading facingPoint(double t0, double t1, double x, double y) {
        return add(t0, t1, HeadingInterpolator.facingPoint(x, y));
    }

    /** Robot heading follows path tangent over [t0, t1]. */
    public PiecewiseHeading tangent(double t0, double t1) {
        return add(t0, t1, HeadingInterpolator.tangent);
    }

    /** Robot heading follows reversed path tangent over [t0, t1]. */
    public PiecewiseHeading reversedTangent(double t0, double t1) {
        return add(t0, t1, HeadingInterpolator.tangent.reverse());
    }

    // ─── Build ────────────────────────────────────────────────────────────────

    /**
     * Builds the final HeadingInterpolator to pass to path.setHeadingInterpolation().
     * Called internally by the commands — you don't need to call this yourself.
     */
    public HeadingInterpolator build() {
        return HeadingInterpolator.piecewise(
                nodes.toArray(new HeadingInterpolator.PiecewiseNode[0])
        );
    }
}