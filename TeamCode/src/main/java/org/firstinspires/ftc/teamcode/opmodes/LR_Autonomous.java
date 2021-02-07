package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.operations.MoveOperation;
import org.firstinspires.ftc.teamcode.operations.Operation;
import org.firstinspires.ftc.teamcode.operations.OperationRunner;
import org.firstinspires.ftc.teamcode.operations.StrafeOperation;
import org.firstinspires.ftc.teamcode.operations.TurnOperation;

@Autonomous(name="Left Red Auto", group="Linear Opmode")
public class LR_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        MoveOperation moveToShoot = new MoveOperation("moving to shoot", robot, 60, 0.5, 10);
        StrafeOperation toFirstShot = new StrafeOperation("moving to first shot", robot, 8, 0.8, 4);
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
        Operation bringBumperBack1 = new Operation("unload ring 1", robot, 1) {
            @Override
            public int operate(double dt) {
                robot.forceUnloadRing();
                return -1;
            }
        };
        StrafeOperation strafeForMidTarget = new StrafeOperation("aiming for mid target", robot, -6, -1.0, 2f);
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
        Operation runRollers = new Operation("loading 3rd ring", robot, 2.5f) {
            @Override
            public int operate(double dt) {
                robot.intake(1);
                return -1;
            }
        };
        StrafeOperation strafeForRightTarget = new StrafeOperation("aiming for left target", robot, -6, -1.0, 2f);
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
        Operation shootRing4 = new Operation("shooting ring 4", robot, 1) {
            @Override
            public int operate(double dt) {
                robot.forceLoadRing();
                return -1;
            }
        };
        Operation bringBumperBack4 = new Operation("unload ring 4", robot, 1) {
            @Override
            public int operate(double dt) {
                robot.forceUnloadRing();
                return -1;
            }
        };
        Operation turnOffFlywheel = new Operation("turning off flywheel", robot, 1) {
            @Override
            public int operate(double dt) {
                robot.setFlywheelPower(0);
                robot.intake(0);
                return 0;
            }
        };
        StrafeOperation strafeBack = new StrafeOperation("lining back up", robot, 8, -1, 0.5f);
        MoveOperation goToLine = new MoveOperation("go to middle zone", robot, 36, 0.5, 10);
        Operation dropWobble = new Operation("drop wobble", robot, 1) {
            public int operate(double dt) {
                robot.lowerWobble();
                return -1;
            }
        };
        Operation raiseWobble = new Operation("raise slide", robot, 1) {
            public int operate(double dt) {
                robot.setSlidePower(1);
                return -1;
            }
        };
        Operation stopWobble = new Operation("stop slide", robot, 1) {
            public int operate(double dt) {
                robot.setSlidePower(0);
                return 0;
            }
        };

        OperationRunner opRun = new OperationRunner(moveToShoot);

        Operation[] operations = {
                moveToShoot, toFirstShot, setFlyPower, shootRing1, bringBumperBack1, strafeForMidTarget,
                shootRing2, bringBumperBack2, runRollers, strafeForRightTarget,
                shootRing3, bringBumperBack3, shootRing4, bringBumperBack4, turnOffFlywheel,
                strafeBack, goToLine, dropWobble, raiseWobble, stopWobble
        };

        for (int i = 0; i < operations.length-1; i++)
            operations[i].linkOperation(operations[i+1]);


        waitForStart();
        runtime.reset();
        double last = runtime.milliseconds();
        while (opModeIsActive()) {
            telemetry.addData("Robot Angle: ",robot.getAngle());
            double now = runtime.milliseconds();
            double dt = (now - last) / 1000.0;
            last = now;

            telemetry.addData("Delta Time", dt);

            opRun.operate(robot, dt);

            robot.printEncoderPositions(telemetry);
            telemetry.addData("Runner Status",opRun.getCurrentDisplay());

            telemetry.update();
        }
    }
}
