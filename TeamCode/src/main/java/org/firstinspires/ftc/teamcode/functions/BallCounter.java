package org.firstinspires.ftc.teamcode.functions;

public class BallCounter {
    private int count;

    public BallCounter(int initialCount) {
        this.count = initialCount;
    }

    public void increment() {
        count++;
    }

    public void decrement() {
        count--;
    }

    public int getCount() {
        return count;
    }

    public void setCount(int count) {
        this.count = count;
    }
}