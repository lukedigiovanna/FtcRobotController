package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.utils.*;
import org.firstinspires.ftc.teamcode.utils.Timer;

import java.util.*;

public abstract class Operation {
    
    private List<Operation> linkedOperations;
    private String display;
    private float maxRuntime;
    private Timer timer;

    public Operation(String display, float maxRuntime) {
        this.display = display;
        this.maxRuntime = maxRuntime;
        this.linkedOperations = new ArrayList<Operation>();
        this.timer = new Timer();
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
    public abstract int operate();

    /**
     * Adds the operation to the end of the list.
     * The first one will be of index 0
     * @param op
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

    public String getDisplay() {
        return this.display;
    }
}
