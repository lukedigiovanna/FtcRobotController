package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.operations.MoveOperation;
import org.firstinspires.ftc.teamcode.operations.Operation;
import org.firstinspires.ftc.teamcode.operations.OperationRunner;
import org.firstinspires.ftc.teamcode.operations.StrafeOperation;
import org.firstinspires.ftc.teamcode.operations.TurnOperation;

@Autonomous(name="Right Red Auto", group="Linear Opmode")
public class RR_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        MoveOperation moveToShoot = new MoveOperation("moving to shoot", robot, 60, 0.5, 10);
        StrafeOperation strafeToShoot = new StrafeOperation("strafing to shoot", robot, -22, 0.5, 10);
        Operation setFlyPower = new Operation("setting power", robot, 1) {
            public int operate(double dt) {
                robot.setFlywheelPower(RobotHardware.DEFAULT_FLYWHEEL_POWER);
                return -1;
            }
        };
        Operation shootRing1 = new Operation("shooting ring 1", robot, 1) {
            @Override
            public int operate(double dt) {
                robot.forceLoadRing();
                return -1;
            }
        };
        Operation bringBumperBack1 = new Operation("unload ring 1", robot, 2) {
            @Override
            public int operate(double dt) {
                robot.forceUnloadRing();
                return -1;
            }
        };
        Operation shootRing2 = new Operation("shooting ring 2", robot, 1) {
            @Override
            public int operate(double dt) {
                robot.forceLoadRing();
                return -1;
            }
        };
        Operation bringBumperBack2 = new Operation("unload ring 2", robot, 1) {
            @Override
            public int operate(double dt) {
                robot.forceUnloadRing();
                return -1;
            }
        };
        Operation runRollers = new Operation("loading 3rd ring", robot, 3) {
            @Override
            public int operate(double dt) {
                robot.intake(1);
                return -1;
            }
        };
        Operation stopRollers = new Operation("stop rollers", robot, 0.2f) {
            @Override
            public int operate(double dt) {
                robot.intake(0);
                return 0;
            }
        };
        Operation shootRing3 = new Operation("shooting ring 3", robot, 1) {
            @Override
            public int operate(double dt) {
                robot.forceLoadRing();
                return -1;
            }
        };
        Operation bringBumperBack3 = new Operation("unload ring 3", robot, 1) {
            @Override
            public int operate(double dt) {
                robot.forceUnloadRing();
                return -1;
            }
        };
        MoveOperation goToZone = new MoveOperation("to zone", robot, 36, 0.6, 10);
        Operation dropWobble = new Operation("drop wobble", robot, 1) {
            public int operate(double dt) {
                robot.setFlywheelPower(0);
                robot.lowerWobble();
                return -1;
            }
        };
        Operation raiseWobble = new Operation("raise slide", robot, 1) {
            public int operate(double dt) {
                robot.setSlidePower(0.5);
                return -1;
            }
        };
        Operation stopWobble = new Operation("stop slide", robot, 0.2f) {
            public int operate(double dt) {
                robot.setSlidePower(0);
                return 0;
            }
        };
        Operation toLine = new MoveOperation("parking", robot, -22, 0.6, 5f);

        OperationRunner opRun = new OperationRunner(moveToShoot);

        Operation[] operations = {
          moveToShoot, strafeToShoot, setFlyPower, shootRing1, bringBumperBack1, shootRing2, bringBumperBack2, runRollers,
          stopRollers, shootRing3, bringBumperBack3, goToZone, dropWobble,  raiseWobble, stopWobble, toLine
        };

        for (int i = 0; i < operations.length - 1; i++)
            operations[i].linkOperation(operations[i+1]);

        waitForStart();
        runtime.reset();
        double last = runtime.milliseconds();
        while (opModeIsActive()) {
            telemetry.addData("Robot Angle: ", robot.getAngle());
            double now = runtime.milliseconds();
            double dt = (now - last) / 1000.0;
            last = now;

            telemetry.addData("Delta Time", dt);

            opRun.operate(robot, dt);

            robot.printEncoderPositions(telemetry);
            telemetry.addData("Runner Status", opRun.getCurrentDisplay());

            telemetry.update();
        }
    }
}
