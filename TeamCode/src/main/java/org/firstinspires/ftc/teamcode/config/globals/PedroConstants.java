package org.firstinspires.ftc.teamcode.config.globals;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * PedroConstants — configures the Pedro Pathing follower for this robot.
 *<p>
 * To use, call PedroConstants.createFollower(hardwareMap) from your OpMode.
 * All values here were empirically tuned for the current drivetrain and field.
 *<p>
 * ── What to tune ──────────────────────────────────────────────────────────────
 *   followerConstants  — PID gains, mass, zero-power deceleration rates
 *   driveConstants     — motor names and measured max velocities (xVelocity / yVelocity)
 *   localizerConstants — pod offsets from robot center, encoder directions
 *   pathConstraints    — global max velocity and acceleration for path following
 *<p>
 * ── Localizer ─────────────────────────────────────────────────────────────────
 *   Currently configured for GoBilda Pinpoint (pinpointLocalizer).
 */
public class PedroConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.75)
            .forwardZeroPowerAcceleration(-29.450011125114425)
            .lateralZeroPowerAcceleration(-66.46948014751574)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.0, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0075, 0, 0, 0.6, 0.025))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.98, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(76.04715424673259)
            .yVelocity(61.40360365922963)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.125)
            .strafePodX(-2.375)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    /**
     * Builds and returns a fully configured Pedro Pathing Follower.
     * Call this once in your OpMode's initialize() and store the result.
     *
     * @param hardwareMap the OpMode's hardwareMap
     * @return a ready-to-use Follower with mecanum drivetrain + Pinpoint localizer
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
