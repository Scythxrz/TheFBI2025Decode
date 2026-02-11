package org.firstinspires.ftc.teamcode.teleop.pathing;

import java.util.ArrayList;
import java.util.List;

public class ObstacleFactory {

    public static Obstacle createBox(
            double centerX,
            double centerY,
            double width,
            double height,
            double clearance
    ) {
        double hw = width / 2.0 + clearance;
        double hh = height / 2.0 + clearance;

        return new Obstacle(
                centerX - hw,
                centerX + hw,
                centerY - hh,
                centerY + hh
        );
    }

    public static List<Obstacle> fieldObstacles() {
        List<Obstacle> obstacles = new ArrayList<>();

        // Example: center field box
        obstacles.add(
                createBox(0, 0, 0, 0 ,0)
        );

        return obstacles;
    }
}
