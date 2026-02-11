package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.functions.BallCounter;
import org.firstinspires.ftc.teamcode.functions.Drawing;
import org.firstinspires.ftc.teamcode.functions.Intake;
import org.firstinspires.ftc.teamcode.functions.Limelight;
import org.firstinspires.ftc.teamcode.functions.Shooter;
import org.firstinspires.ftc.teamcode.autonomous.pathSequence.autonomousBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.FileWriter;
import java.io.IOException;
@Autonomous(name = "FBI Auto")
public class FBIAuto extends OpMode {
    // Define Blue Alliance coordinates ONLY.
    private final double[] smallStart = {47.212547461231765, 8, 270}, bigStart = {0, 0, 0};
    private final double[] score = {53.95941530900375, 85.48144609472519, 314.1272869639285}, //-75.25, 53.25, 41},
            PPGStart = {54.1340178620867, 76.76611203461731, 180}, PPGFinish = {19.448813367811418, 76.76611203461731, 180},
            PGPStart = {50.36182786438276, 52.59734645219842, 180}, PGPFinish = {22.282476495990146, 52.59734645219842, 180},
            GPPStart = {45.47193901092421, 29.53032434416428, 180}, GPPFinish = {22.984356409285184, 29.78758489762085, 180},
            gateStart = {0, 0, 0}, gateFinish = {0, 0, 0},
            end = {21.99122886331602, 51.470524875238375, 151.64063259579282};

    // Special coordinates for the ALLIANCE_CLOSE sequence (Blue side)
    private final double[] smallShoot = {55.78071467999224, 14.52857702232174, 296.34474304417836};
    private final double[] load1 = {28.86, 5.37, 195.19}, load2 = {10.32, 5.11, 196.68},
            load3 = {19.95, 3.19, 180}, load4 = {8.27, 3.21, 180},
            smallEnd = {23.32, 6.84, 121.83};

    // 18 arti coords

    private double[] start = {32, 135, 270};
    private final double[] score18 = {54.487, 80.887, 312};
    private final double[] pgp11 = {54.487, 80.887};
    private final double[] pgp12 = {41.794, 59.336};
    private final double[] pgp13 = {20.334, 60.906};
    private final double[] ppg11 = {54.487, 80.887, 312};
    private final double[] ppg12 = {46.802, 82.999, 180};
    private final double[] ppg21 = {23.573, 84.442, 180};
    private final double[] gate11 = {54.487, 80.887, 312};
    private final double[] gate12 = {39.218, 50.323};
    private final double[] gate13 = {19.331, 60.73, 142};
    private final double[] gate21 = {9.731, 60.72, 141};
    private final double[] gpp11 = {54.487, 80.887};
    private final double[] gpp12 = {75.662, 37.495};
    private final double[] gpp13 = {15.008, 39.966};
    private final double[] end18 = {56.487, 110.887, 333};

    private final double[] farstart = {50.5, 8, 270}, farscore = {57.5, 15, 290}, farend = {40.5, 8, 270}, far1 = {15, 20, 210}, far2 = {15, 10, 210}, /*far3 = {35, 9, 180}, far4 = {12, 9, 180}*/ far3 = {35, 15, 215}, far4 = {12, 9, 180}, fargpp1 = {46.802, 32.6, 180}, fargpp2 = {23.573, 32.6, 180};
    /*
    private final double[] start = {32.417, 138.485, 270}, score18 = {52.857, 84.950, 315},
            pgp11 = {58.857, 84.950}, pgp12 = {68.176, 52.166}, pgp13 = {20.504, 53.602},
            pgp21 = {20.504, 53.602, 180}, pgp22 = {66.176, 56.166}, pgp23 = {52.857, 84.950, 315},
            gate11 = {58.857, 81.950, 315}, gate12 = {55.122, 48.398}, gate13 = {8.413, 58.251, 143},
            gate21 = {8.413, 58.251, 145}, gate22 = {55.122, 48.398}, gate23 = {52.857, 84.950, 315},
            gpp11 = {58.857, 83.950, 315}, gpp12 = {48, 83.950, 180}, gpp13 = {22.687, 83.651, 180},
            gpp21 = {22.687, 83.651, 180}, gpp22 = {52.857, 84.950, 315},
            ppg11 = {58.857, 83.950}, ppg12 = {69.313, 31.071}, ppg13 = {15.056, 30.253},
            ppg21 = {15.056, 30.253, 180}, ppg22 = {52.857, 103.433, 320};*/

    private Pose toPose(double[] array) {
        double heading = 0;
        if (array.length >= 3) {
            heading = Math.toRadians(array[2]);
        }
        return new Pose(array[0], array[1], heading);
    }
    private enum StartPosition {
        RED_CLOSE("Red Small"),
        RED_FAR("Red Big"),
        BLUE_CLOSE("Blue Small"),
        BLUE_FAR("Blue Big");
        public final String displayName;
        StartPosition(String displayName) { this.displayName = displayName; }
    }

    private enum SequenceType {
        fifteenfull("15 1 gate cycle"),
        fifteenpartial("15 2 gate cycle"),
        far9("9 ball far"),
        far9gpp("9 ball far w/ gpp");
        public final String displayName;
        SequenceType(String displayName) { this.displayName = displayName; }
    }

    private StartPosition selectedStart = StartPosition.RED_CLOSE;
    private SequenceType selectedSequence = SequenceType.fifteenfull;
    private boolean upPressedLast, downPressedLast, bPressedLast, aPressedLast;
    private int menuStage = 0;
    private double power;
    private int allianceTagId;
    private Follower follower;
    private autonomousBuilder builder;
    private BallCounter ballCounter;
    private boolean includeGate = false;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Shooter.INSTANCE.initialize(hardwareMap);
        Intake.INSTANCE.initialize(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap);
        ballCounter = new BallCounter(3);
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (menuStage == 0) { // Position selection
            if (gamepad1.dpad_up && !upPressedLast) selectedStart = StartPosition.values()[(selectedStart.ordinal() - 1 + 4) % 4];
            if (gamepad1.dpad_down && !downPressedLast) selectedStart = StartPosition.values()[(selectedStart.ordinal() + 1) % 4];
            if (gamepad1.b && !bPressedLast) menuStage = 1;
        } else if (menuStage == 1) { // Sequence selection
            if (gamepad1.dpad_up && !upPressedLast) selectedSequence = SequenceType.values()[(selectedSequence.ordinal() - 1 + SequenceType.values().length) % SequenceType.values().length];
            if (gamepad1.dpad_down && !downPressedLast) selectedSequence = SequenceType.values()[(selectedSequence.ordinal() + 1) % SequenceType.values().length];
            if (gamepad1.b && !bPressedLast) menuStage = 0;
        }

        upPressedLast = gamepad1.dpad_up;
        downPressedLast = gamepad1.dpad_down;
        bPressedLast = gamepad1.b;
        aPressedLast = gamepad1.a;

        telemetry.addLine("=== FBI Enhanced Auto ===");
        telemetry.addData("Starting Position", selectedStart.displayName);
        telemetry.addData("Sequence Type", selectedSequence.displayName);
        telemetry.addLine(menuStage == 0 ? "\nPress B to select sequence" : "\nPress B to return to sequence selection");
        telemetry.update();
    }

    @Override
    public void start() {
        boolean isBlue = selectedStart == StartPosition.BLUE_CLOSE || selectedStart == StartPosition.BLUE_FAR;
        boolean isClose = selectedStart == StartPosition.BLUE_CLOSE || selectedStart == StartPosition.RED_CLOSE;
        allianceTagId = isBlue ? 24 : 20; // DECODE Tag IDs

        double[] startCoords = isClose ? smallStart : bigStart;
        double[] scoreCoords = score, endCoords = end;
        double[] start1 = PPGStart, finish1 = PPGFinish, start2 = PGPStart, finish2 = PGPFinish, start3 = GPPStart, finish3 = GPPFinish;
        double[] gateFirst = gateStart, gateEnd = gateFinish;
        power = 0.7;
        // if (!isBlue) start = new double[]{30, 134, 270};

        /*if (selectedSequence == SequenceType.ALLIANCE_CLOSE) {
            scoreCoords = smallShoot;
            endCoords = smallEnd;
            power = 1;
        }*/
        if (selectedSequence == SequenceType.far9 || selectedSequence == SequenceType.far9gpp) {
            startCoords = farstart;
            scoreCoords = farscore;
            endCoords = farend;
        } else {
            startCoords = start;
            scoreCoords = score18;
            endCoords = end18;
        }

        builder = new autonomousBuilder(follower, telemetry, isBlue, ballCounter)
                .withCoordinateArrays(startCoords, scoreCoords, start1, finish1, start2, finish2, start3, finish3, gateStart, gateEnd, endCoords);

        /*if (selectedSequence == SequenceType.ALLIANCE_CLOSE) builder.scoreFromStart(power, 3000L, true);
        else builder.scoreFromStart(power, 3500L, false);*/
        switch (selectedSequence) {
            case fifteenfull:
                builder.scoreFromStart(1850, 1750);
                builder.customPath(toPose(pgp11), new Pose[]{toPose(pgp12), toPose(pgp13)}, true, false, 1, false, true).customPath(toPose(score18), false, true, 1).score(1850, 1750);
                builder.customPath(toPose(gate11), new Pose[]{toPose(gate12), toPose(gate13)}, true, false, 1, false, false).customPath(toPose(gate21), true, false, 1).wait(750, true).customPath(toPose(score18), false, true, 1).score(1850, 1750);
                builder.customPath(toPose(ppg11), toPose(ppg12), true, false, 1).customPath(toPose(ppg21), true, false, 1).customPath(toPose(score18), false, true, 1).score(1850, 1750);
                builder.customPath(toPose(gpp11), new Pose[]{toPose(gpp12), toPose(gpp13)}, true, false, 1, false, true).customPath(toPose(end18), false, true, 1).score(1700, 1750);
                break;
            case fifteenpartial:
                builder.scoreFromStart(1850, 1750);
                builder.customPath(toPose(pgp11), new Pose[]{toPose(pgp12), toPose(pgp13)}, true, false, 1, false, true).customPath(toPose(score18), false, true, 1).score(1850, 1750);
                builder.customPath(toPose(gate11), new Pose[]{toPose(gate12), toPose(gate13)}, true, false, 1, false, false).customPath(toPose(gate21), true, false, 1).wait(750, true).customPath(toPose(score18), false, true, 1).score(1850, 1750);
                builder.customPath(toPose(gate11), new Pose[]{toPose(gate12), toPose(gate13)}, true, false, 1, false, false).customPath(toPose(gate21), true, false, 1).wait(750, true).customPath(toPose(score18), false, true, 1).score(1850, 1750);
                builder.customPath(toPose(ppg11), toPose(ppg12), true, false, 1).customPath(toPose(ppg21), true, false, 1).customPath(toPose(end18), false, true, 1).score(1850, 1750);
                break;
            case far9:
                builder.scoreFromStart(2450, 3000).customPath(toPose(far1), true, false, 1);
                builder.customPath(toPose(far2), true, false, 0.75).customPath(toPose(farscore), false, true, 1).score(2450, 3000);
                builder.customPath(toPose(far3), true, false, 1).customPath(toPose(far4), true, false, 0.75);
                builder.customPath(toPose(farscore), false, true, 1).score(2450, 3000);
                builder.customPath(toPose(far3), true, false, 1).customPath(toPose(far4), true, false, 0.75);
                builder.customPath(toPose(farscore), false, true, 1).score(2450, 3000);
                break;
            case far9gpp:
                builder.scoreFromStart(2450, 3000);
                builder.customPath(toPose(fargpp1), true, false, 1).customPath(toPose(fargpp2), true, false, 1);
                builder.customPath(toPose(farscore), false, true, 1).score(2450, 3000);
                builder.customPath(toPose(far1), true, false, 1).customPath(toPose(far2), true, false, 1);
                builder.customPath(toPose(farscore), false, true, 1).score(2450, 3000);
                builder.customPath(toPose(far3), true, false, 1).customPath(toPose(far4), true, false, 0.75);
                builder.customPath(toPose(farscore), false, true, 1).score(2450, 3000);
                break;

        }

        builder.finish().start();
    }

    @Override
    public void loop() {
        follower.update();
        builder.update();
        Drawing.drawDebug(follower);

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Complete", builder.isFinished());
        telemetry.addData("Ball Count", ballCounter.getCount());
        telemetry.addData("Shooter Vel", Shooter.INSTANCE.getVelocity());
        telemetry.addData("Target Vel", Shooter.INSTANCE.getTVelocity());
        telemetry.addData("Intake Vel", Intake.INSTANCE.getVelocity());
        telemetry.addData("Ball Detection", Shooter.INSTANCE.ballDetection());
        telemetry.addData("Is Blue?", selectedStart == StartPosition.BLUE_CLOSE || selectedStart == StartPosition.BLUE_FAR);

        //telemetry.addData("Vel Diff", Math.abs(Shooter.INSTANCE.getVelocity() - Shooter.INSTANCE.getTVelocity()));
        telemetry.update();
    }

    @Override
    public void stop() {
        savePose(follower.getPose());
    }

    private void savePose(Pose pose) {
        try (FileWriter writer = new FileWriter("/sdcard/FIRST/pose.txt")) {
            writer.write(pose.getX() + "," + pose.getY() + "," + pose.getHeading());
        } catch (IOException e) {
            telemetry.addLine("Failed to save pose");
        }
    }
}
