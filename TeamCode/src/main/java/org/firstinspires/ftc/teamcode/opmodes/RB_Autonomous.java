package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.operations.ActivateFlywheelOperation;
import org.firstinspires.ftc.teamcode.operations.ActivateIntakeOperation;
import org.firstinspires.ftc.teamcode.operations.DeactivateFlywheelOperation;
import org.firstinspires.ftc.teamcode.operations.LoadRingOperation;
import org.firstinspires.ftc.teamcode.operations.LowerWobbleOperation;
import org.firstinspires.ftc.teamcode.operations.MoveOperation;
import org.firstinspires.ftc.teamcode.operations.Operation;
import org.firstinspires.ftc.teamcode.operations.OperationRunner;
import org.firstinspires.ftc.teamcode.operations.RaiseWobbleOperation;
import org.firstinspires.ftc.teamcode.operations.ShootRingOperation;
import org.firstinspires.ftc.teamcode.operations.StrafeOperation;
import org.firstinspires.ftc.teamcode.operations.TurnOperation;
import org.firstinspires.ftc.teamcode.operations.WaitOperation;

@Autonomous(name="Right Blue Auto", group="Linear Opmode")
public class RB_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);
        Operation.setRobot(robot);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Operation putBumperBack = new LoadRingOperation();
        Operation park = new MoveOperation("Parking", 15, 1.0, 10f, putBumperBack);
        Operation stopFlywheel = new DeactivateFlywheelOperation(park);
        Operation shootRing4 = new ShootRingOperation(stopFlywheel);
        Operation waitFor3 = new WaitOperation("Waiting", 1.0f, shootRing4);
        Operation activateIntake = new ActivateIntakeOperation(waitFor3);
        Operation loadRing4 = new LoadRingOperation(activateIntake);
        Operation shootRing3 = new ShootRingOperation(loadRing4);
        Operation loadRing3 = new LoadRingOperation(shootRing3);
        Operation shootRing2 = new ShootRingOperation(loadRing3);
        Operation loadRing2 = new LoadRingOperation(shootRing2);
        Operation shootRing1 = new ShootRingOperation(loadRing2);
        Operation straight = new TurnOperation("Straight", 0, 0.6, 10f, shootRing1);
        Operation powerFlywheel = new ActivateFlywheelOperation(straight);

        Operation turnLeft = new TurnOperation("Turn towards goal", 344, 0.5, 10f, powerFlywheel);
        Operation goBack = new MoveOperation("To Launch Line", -40, 0.65, 10f, turnLeft);
        Operation raiseWobble = new RaiseWobbleOperation(goBack);
        Operation lowerWobble = new LowerWobbleOperation(raiseWobble);
        Operation moveToZone = new MoveOperation("Move to B zone", 104, 0.65, 10f, lowerWobble);

        OperationRunner opRun = new OperationRunner(moveToZone);

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

            telemetry.addData("Runner Status",opRun.getCurrentDisplay());
            robot.printInformation(telemetry);

            telemetry.update();
        }
    }
}
