package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.Timer;

import java.util.*;

public abstract class Operation {
    
    private List<Operation> linkedOperations;
    private String displayName;
    private float maxRuntime;
    private Timer timer;
    protected RobotHardware robot;

    public Operation(String displayName, RobotHardware robot, float maxRuntime) {
        this.displayName = displayName;
        this.maxRuntime = maxRuntime;
        this.linkedOperations = new ArrayList<Operation>();
        this.timer = new Timer();
        this.robot = robot;
    }

    /**
     * Returns true if there is at least one linked operation
     * @return
     */
    public boolean hasNext() {
        return this.linkedOperations.size() > 0;
    }

    /**
     * First thing to be called when an operation starts.
     * Resets the timer by default.
     */
    public void init() {
        this.timer.hold();
    }

    /**
     * Return of -1 means the operation is still running
     * Return of 0 to ... indicates the next operation in the list to run
     * @return
     */
    public abstract int operate(double dt);

    /**
     * Adds the operation to the end of the list.
     * The first one will be of index 0
     */
    public void linkOperation(Operation op) {
        this.linkedOperations.add(op);
    }

    public boolean isTimerDone() {
        return this.timer.elapsed() >= this.maxRuntime;
    }

    public Operation get(int index) {
        return linkedOperations.get(index);
    }

    public String getDisplayName() {
        return this.displayName;
    }
}
