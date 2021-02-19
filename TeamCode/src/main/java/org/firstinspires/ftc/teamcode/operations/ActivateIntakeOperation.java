package org.firstinspires.ftc.teamcode.operations;

public class ActivateIntakeOperation extends Operation {
    public ActivateIntakeOperation(Operation... futureOps) {
        super("Activate Intake", 0f, futureOps);
    }

    @Override
    public int operate(double dt) {
        robot.intake(1.0);
        return -1;
    }
}
