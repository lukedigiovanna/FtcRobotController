package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.operations.MoveOperation;
import org.firstinspires.ftc.teamcode.operations.Operation;
import org.firstinspires.ftc.teamcode.operations.OperationRunner;

@Autonomous(name="Left Blue Auto", group="Linear Opmode")
public class LB_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        MoveOperation moveToShoot = new MoveOperation("moving to shoot", robot, 60, 0.5, 10);
        Operation setFlyPower = new Operation("setting power", robot, 1) {
            public int operate() {
                robot.setFlywheelPower(RobotHardware.DEFAULT_FLYWHEEL_POWER);
                return -1;
            }
        };
        Operation shootRing1 = new Operation("shooting ring 1", robot, 1) {
            @Override
            public int operate() {
                robot.forceLoadRing();
                return -1;
            }
        };
        Operation bringBumperBack1 = new Operation("unload ring 1", robot, 2) {
            @Override
            public int operate() {
                robot.forceUnloadRing();
                return -1;
            }
        };
        Operation shootRing2 = new Operation("shooting ring 2", robot, 1) {
            @Override
            public int operate() {
                robot.forceLoadRing();
                return -1;
            }
        };
        Operation bringBumperBack2 = new Operation("unload ring 2", robot, 2) {
            @Override
            public int operate() {
                robot.forceUnloadRing();
                return -1;
            }
        };
        MoveOperation goToLine = new MoveOperation("parking", robot, 14, 0.5, 10);
        Operation dropWobble = new Operation("drop wobble", robot, 4) {
            public int operate() {
                robot.setFlywheelPower(0);
                robot.raiseWobble();
                return -1;
            }
        };

        OperationRunner opRun = new OperationRunner(moveToShoot);
        moveToShoot.linkOperation(setFlyPower);
        setFlyPower.linkOperation(shootRing1);
        shootRing1.linkOperation(bringBumperBack1);
        bringBumperBack1.linkOperation(shootRing2);
        shootRing2.linkOperation(bringBumperBack2);
        bringBumperBack2.linkOperation(goToLine);
        goToLine.linkOperation(dropWobble);
        waitForStart();
        runtime.reset();
        double last = runtime.milliseconds();
        while (opModeIsActive()) {
            telemetry.addData("Robot Angle: ",robot.getAngle());
            double now = runtime.milliseconds();
            double dt = (now - last) / 1000.0;
            last = now;

            telemetry.addData("Delta Time", dt);

            opRun.operate(robot);

            robot.printEncoderPositions(telemetry);
            telemetry.addData("Runner Status",opRun.getCurrentDisplay());

            telemetry.update();
        }
    }
}
