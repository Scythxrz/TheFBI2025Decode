package org.firstinspires.ftc.teamcode.teleop.pathing;

import com.pedropathing.geometry.Pose;

public class Obstacle {

    public double xmin, xmax, ymin, ymax;

    public Obstacle(double xmin, double xmax, double ymin, double ymax) {
        this.xmin = xmin;
        this.xmax = xmax;
        this.ymin = ymin;
        this.ymax = ymax;
    }

    public boolean contains(Pose p) {
        return p.getX() >= xmin && p.getX() <= xmax &&
                p.getY() >= ymin && p.getY() <= ymax;
    }
}
