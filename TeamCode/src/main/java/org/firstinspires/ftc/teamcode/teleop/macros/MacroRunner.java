package org.firstinspires.ftc.teamcode.teleop.macros;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class MacroRunner {
    private final Queue<MacroAction> actionQueue;
    private MacroAction currentAction;
    private boolean isRunning = false;

    public MacroRunner(List<MacroAction> actions) {
        this.actionQueue = new LinkedList<>(actions);
    }

    public void start() {
        if (!actionQueue.isEmpty()) {
            isRunning = true;
            startNextAction();
        }
    }

    public void cancel() {
        isRunning = false;
        currentAction = null;
        actionQueue.clear();
        // Optional: Stop motors/follower here if needed
    }

    public void update() {
        if (!isRunning || currentAction == null) return;

        currentAction.update();

        if (currentAction.isFinished()) {
            startNextAction();
        }
    }

    private void startNextAction() {
        currentAction = actionQueue.poll();
        if (currentAction != null) {
            currentAction.start();
        } else {
            isRunning = false; // Sequence complete
        }
    }

    public boolean isBusy() {
        return isRunning;
    }
}
