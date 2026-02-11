package org.firstinspires.ftc.teamcode.teleop.macros;

public interface MacroAction {
    void start();       // Called once when the action begins
    void update();      // Called every loop
    boolean isFinished(); // Returns true when done
}
