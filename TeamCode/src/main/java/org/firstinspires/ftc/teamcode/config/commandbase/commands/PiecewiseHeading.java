package org.firstinspires.ftc.teamcode.config.commandbase.commands;

import com.pedropathing.paths.HeadingInterpolator;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * PiecewiseHeading — fluent builder for piecewise heading interpolation.
 *
 * Lets you specify different heading behaviors across segments of a single path,
 * identified by t-values from 0.0 (path start) to 1.0 (path end).
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *
 *   PiecewiseHeading heading = new PiecewiseHeading()
 *       .add(0.0, 0.4, HeadingInterpolator.linear(0, Math.toRadians(180)))
 *       .add(0.4, 0.7, HeadingInterpolator.facingPoint(72, 72))
 *       .add(0.7, 1.0, HeadingInterpolator.tangent);
 *
 *   // Then pass to any command:
 *   new DriveToPose(follower, target, heading)
 *   new WindUpAndDrive(follower, target, 1850, heading, 1.0)
 *   new MoveAndShoot(follower, target, 3, 1850, isBlue, FiringMode.RAPID, heading)
 *   new ShootWhileMoving(follower, target, 3, 1850, isBlue, FiringMode.RAPID, heading)
 *
 * ── Segment shortcuts ─────────────────────────────────────────────────────────
 *
 *   .linear(t0, t1, fromRad, toRad)          → HeadingInterpolator.linear(...)
 *   .reversedLinear(t0, t1, fromRad, toRad)  → HeadingInterpolator.reversedLinear(...)
 *   .facingPoint(t0, t1, x, y)               → HeadingInterpolator.facingPoint(...)
 *   .tangent(t0, t1)                          → HeadingInterpolator.tangent
 *   .add(t0, t1, interpolator)               → any HeadingInterpolator directly
 *
 * ── Notes ─────────────────────────────────────────────────────────────────────
 *   Segments must cover 0.0–1.0 with no gaps and no overlaps.
 *   Pedro will throw if the piecewise nodes don't tile cleanly.
 */
public class PiecewiseHeading {

    private final List<HeadingInterpolator.PiecewiseNode> nodes = new ArrayList<>();
    // Lazy: start heading resolved at build() time (inside initialize()), not construction time
    private final List<double[]>       lazySegments        = new ArrayList<>(); // {t0, t1, toRad}
    private final List<DoubleSupplier> lazyFrom            = new ArrayList<>();
    // Lazy shortest: same but normalizes toRad to shortest rotational path from fromRad
    private final List<double[]>       lazyShortestSegments = new ArrayList<>(); // {t0, t1, toRad}
    private final List<DoubleSupplier> lazyShortestFrom     = new ArrayList<>();

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


    /**
     * Linear heading where the START angle is supplied lazily at build() time.
     * build() is called from DriveToPose.initialize() when the command actually
     * starts executing — so the supplier sees the robot's live heading at that
     * moment, not the stale heading from when the sequence was constructed.
     *
     * Use whenever the start heading is "wherever the robot ended up after the
     * previous path" e.g. gate intakes after a scoring run.
     *
     *   .lazyLinear(0.6, 1.0, follower::getHeading, Math.toRadians(135))
     */
    public PiecewiseHeading lazyLinear(double t0, double t1, DoubleSupplier fromRad, double toRad) {
        lazySegments.add(new double[]{t0, t1, toRad});
        lazyFrom.add(fromRad);
        return this;
    }


    /**
     * Like lazyLinear, but always takes the shortest rotational path to toRad.
     *
     * Pedro's linear() interpolates raw radians — if the robot arrives with a heading
     * numerically far from toRad (e.g. 491° vs 135°, same physical angle), it spins
     * the long way. This normalizes toRad into [fromRad - π, fromRad + π] at build()
     * time so the robot always turns the short way regardless of accumulated rotation.
     *
     *   .lazyLinearShortest(0.6, 1.0, follower::getHeading, Math.toRadians(135))
     */
    public PiecewiseHeading lazyLinearShortest(double t0, double t1, DoubleSupplier fromRad, double toRad) {
        lazyShortestSegments.add(new double[]{t0, t1, toRad});
        lazyShortestFrom.add(fromRad);
        return this;
    }

    /** Reversed linear heading from fromRad to toRad over [t0, t1]. */
    public PiecewiseHeading reversedLinear(double t0, double t1, double fromRad, double toRad) {
        return add(t0, t1, HeadingInterpolator.reversedLinear(fromRad, toRad));
    }

    /** Robot faces a fixed field point (x, y) over [t0, t1] — front of robot toward point. */
    public PiecewiseHeading facingPoint(double t0, double t1, double x, double y) {
        return add(t0, t1, HeadingInterpolator.facingPoint(x, y));
    }

    /**
     * Robot faces AWAY from a fixed field point (x, y) over [t0, t1].
     * Use this when the back of the robot needs to point toward the target
     * (e.g. rear-mounted shooter aiming at the goal).
     */
    public PiecewiseHeading facingAwayFromPoint(double t0, double t1, double x, double y) {
        return add(t0, t1, HeadingInterpolator.facingPoint(x, y).reverse());
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
        // Resolve lazy linear segments
        for (int i = 0; i < lazySegments.size(); i++) {
            double[] seg   = lazySegments.get(i);
            double fromRad = lazyFrom.get(i).getAsDouble();
            nodes.add(new HeadingInterpolator.PiecewiseNode(
                    seg[0], seg[1], HeadingInterpolator.linear(fromRad, seg[2])));
        }
        // Resolve lazy shortest-path linear segments
        for (int i = 0; i < lazyShortestSegments.size(); i++) {
            double[] seg   = lazyShortestSegments.get(i);
            double fromRad = lazyShortestFrom.get(i).getAsDouble();
            double toRad   = seg[2];
            // Normalize toRad into [fromRad - π, fromRad + π] — shortest rotational path
            double diff = ((toRad - fromRad + Math.PI) % (2 * Math.PI)) - Math.PI;
            double normalizedTo = fromRad + diff;
            nodes.add(new HeadingInterpolator.PiecewiseNode(
                    seg[0], seg[1], HeadingInterpolator.linear(fromRad, normalizedTo)));
        }
        return HeadingInterpolator.piecewise(
                nodes.toArray(new HeadingInterpolator.PiecewiseNode[0])
        );
    }
}