package org.firstinspires.ftc.teamcode.operations;

public class LowerSlideOperation extends Operation {
    public LowerSlideOperation(Operation... futureOps) {
        super("Raising slide", 0.0f, futureOps);
    }

    public int operate(double dt) {
        robot.setSlidePower(-0.6);
        robot.raiseWobble();
        return -1;
    }
}
