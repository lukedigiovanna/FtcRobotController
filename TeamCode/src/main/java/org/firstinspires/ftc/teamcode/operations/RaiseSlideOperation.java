package org.firstinspires.ftc.teamcode.operations;

public class RaiseSlideOperation extends Operation {
    public RaiseSlideOperation(Operation... futureOps) {
        super("Raising slide", 0.0f, futureOps);
    }

    public int operate(double dt) {
        robot.setSlidePower(0.6);
        return -1;
    }
}
