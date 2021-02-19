package org.firstinspires.ftc.teamcode.operations;

public class RaiseWobbleOperation extends Operation {
    public RaiseWobbleOperation(Operation... futureOps) {
        super("Raising wobble", 1, futureOps);
    }

    @Override
    public int operate(double dt) {
        robot.raiseWobble();
        return -1;
    }
}
