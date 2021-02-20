package org.firstinspires.ftc.teamcode.operations;

public class LoadRingOperation extends Operation {
    public LoadRingOperation(Operation... futureOps) {
        super("Loading ring", 0.4f, futureOps);
    }

    public int operate(double dt) {
        robot.forceUnloadRing();
        return -1;
    }
}
