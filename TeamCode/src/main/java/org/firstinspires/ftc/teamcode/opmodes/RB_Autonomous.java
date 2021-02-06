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

@Autonomous(name="Right Blue Auto", group="Linear Opmode")
public class RB_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        MoveOperation moveToShoot = new MoveOperation("moving to shoot", robot, 60, 0.5, 10);
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
        StrafeOperation strafeForMidTarget = new StrafeOperation("aiming for mid target", robot, 4, -1.0, 0.5f);
        Operation shootRing2 = new Operation("shooting ring 2", robot, 1) {
            @Override
            public int operate(double dt) {
                robot.forceLoadRing();
                return -1;
            }
        };
        Operation bringBumperBack2 = new Operation("unload ring 2", robot, 2) {
            @Override
            public int operate(double dt) {
                robot.forceUnloadRing();
                return -1;
            }
        };
        Operation runRollers = new Operation("loading 3rd ring", robot, 5) {
            @Override
            public int operate(double dt) {
                robot.intake(1);
                return -1;
            }
        };
        StrafeOperation strafeForRightTarget = new StrafeOperation("aiming for left target", robot, 4, -1.0, 0.5f);
        Operation shootRing3 = new Operation("shooting ring 2", robot, 1) {
            @Override
            public int operate(double dt) {
                robot.forceLoadRing();
                return -1;
            }
        };
        Operation bringBumperBack3 = new Operation("unload ring 2", robot, 2) {
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
                return 0;
            }
        };
        StrafeOperation strafeBack = new StrafeOperation("lining back up", robot, -8, -1, 0.5f);
        MoveOperation goToLine = new MoveOperation("parking", robot, 14, 0.5, 10);
        Operation dropWobble = new Operation("drop wobble", robot, 4) {
            public int operate(double dt) {
                robot.raiseWobble();
                return -1;
            }
        };

        OperationRunner opRun = new OperationRunner(moveToShoot);
                                                                                                                                                                                moveToShoot.linkOperation(setFlyPower);
        setFlyPower.linkOperation(shootRing1);
        shootRing1.linkOperation(bringBumperBack1);
        bringBumperBack1.linkOperation(strafeForMidTarget);
        strafeForMidTarget.linkOperation(shootRing2);
        shootRing2.linkOperation(bringBumperBack2);
        bringBumperBack2.linkOperation(strafeForRightTarget);
        strafeForRightTarget.linkOperation(runRollers);
        runRollers.linkOperation(shootRing3);
        shootRing3.linkOperation(bringBumperBack3);
        bringBumperBack3.linkOperation(strafeBack);
        turnOffFlywheel.linkOperation(strafeBack);
        strafeBack.linkOperation(goToLine);
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

            opRun.operate(robot, dt);

            robot.printEncoderPositions(telemetry);
            telemetry.addData("Runner Status",opRun.getCurrentDisplay());

            telemetry.update();
        }
    }
}
