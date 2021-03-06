package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class OperationRunner {
    private Operation currentOperation;
    private boolean hasFirstInitialized = false;

    public OperationRunner(Operation firstNode) {
        this.currentOperation = firstNode;
    }   

    private boolean isOver;

    public void operate(RobotHardware robot, double dt) {
        if (isOver)
            return; // do nothing if we are done operating.

        if (!hasFirstInitialized) {
            currentOperation.init();
            hasFirstInitialized = true;
        }
        int result = currentOperation.operate(dt);
        if (result == -1 && currentOperation.isTimerDone())
            result = 0;
        if (result > -1) { // if the current operation is done, move on to the next one
            robot.stopChassis();
            robot.clearEncoders();
            if (currentOperation.hasNext()) {
                currentOperation = currentOperation.get(result);
                currentOperation.init();
            } else {
                isOver = true;
            }
        }
    }

    public String getCurrentDisplay() {
        if (isOver)
            return "Complete!";
        else
            return this.currentOperation.getDisplayName();
    }
}