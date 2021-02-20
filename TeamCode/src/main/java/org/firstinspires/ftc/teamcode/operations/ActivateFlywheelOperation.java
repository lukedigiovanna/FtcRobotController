package org.firstinspires.ftc.teamcode.operations;

public class ActivateFlywheelOperation extends Operation {
    public ActivateFlywheelOperation(Operation... futureOps) {
        super("Powering flywheel", 0.5f, futureOps);
    }

    @Override
    public int operate(double dt) {
        robot.setFlywheelPower(robot.getRecommendedFlywheelPower());
        return -1;
    }
}
