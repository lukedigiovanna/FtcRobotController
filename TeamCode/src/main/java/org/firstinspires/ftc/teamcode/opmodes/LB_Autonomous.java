package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.operations.ActivateFlywheelOperation;
import org.firstinspires.ftc.teamcode.operations.ActivateIntakeOperation;
import org.firstinspires.ftc.teamcode.operations.DeactivateFlywheelOperation;
import org.firstinspires.ftc.teamcode.operations.DetectOperation;
import org.firstinspires.ftc.teamcode.operations.LoadRingOperation;
import org.firstinspires.ftc.teamcode.operations.LowerSlideOperation;
import org.firstinspires.ftc.teamcode.operations.LowerWobbleOperation;
import org.firstinspires.ftc.teamcode.operations.MoveOperation;
import org.firstinspires.ftc.teamcode.operations.Operation;
import org.firstinspires.ftc.teamcode.operations.OperationRunner;
import org.firstinspires.ftc.teamcode.operations.RaiseSlideOperation;
import org.firstinspires.ftc.teamcode.operations.RaiseWobbleOperation;
import org.firstinspires.ftc.teamcode.operations.RelativeTurnOperation;
import org.firstinspires.ftc.teamcode.operations.ShootRingOperation;
import org.firstinspires.ftc.teamcode.operations.StrafeOperation;
import org.firstinspires.ftc.teamcode.operations.StrictTurnOperation;
import org.firstinspires.ftc.teamcode.operations.TurnOperation;
import org.firstinspires.ftc.teamcode.operations.WaitOperation;

@Autonomous(name="Left Blue Auto", group="Linear Opmode")
public class LB_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);
        Operation.setRobot(robot);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*
        ns_ indicates that these operations are a part of the NONE STACK branch
        ss_ indicates SINGLE STACK
        qs_ indicates QUAD STACK
         */

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

        //region operations to drop the wobble in the NO STACK ZONE
        Operation ns_toLaunch = new MoveOperation("To launch line (ns)", -10, 0.6, 10f, powerFlywheel);
        Operation ns_raiseWobble = new RaiseWobbleOperation(ns_toLaunch);
        Operation ns_lowerWobble = new LowerWobbleOperation(ns_raiseWobble);
        Operation ns_realign = new StrictTurnOperation("Realignment (ns)", 0, 0.25, 10f, ns_lowerWobble);
        Operation ns_strafeRight = new StrafeOperation("Away from wall (ns)", 16, 0.6, 10, ns_realign);
        Operation ns_moveToZone = new MoveOperation("Moving to zone (ns)", 54, 0.6, 10, ns_strafeRight);
        Operation ns_turnToLine = new TurnOperation("Turning to line (ns)", 0, 0.65, 10, ns_moveToZone);
        //endregion

        //region operations to drop the wobble in the SINGLE STACK ZONE
        Operation ss_toLaunch = new MoveOperation("To launch line (ss)", -28, 0.7, 10f, powerFlywheel);
        Operation ss_turnAroundToLaunch = new TurnOperation("Turn Around (ss)", 0, 0.75, 10f, ss_toLaunch);
        Operation ss_raiseWobble = new RaiseWobbleOperation(ss_turnAroundToLaunch);
        Operation ss_lowerWobble = new LowerWobbleOperation(ss_raiseWobble);
        Operation ss_realign = new StrictTurnOperation("Realignment (qs)", 0, 0.25, 10f, ss_lowerWobble);
        Operation ss_turnAround = new TurnOperation("Turn Around (ss)", 180, 0.75, 10f, ss_realign );
        Operation ss_strafeRight = new StrafeOperation("Away from wall (ss)", 12, 0.6, 10, ss_turnAround);
        Operation ss_moveToZone = new MoveOperation("Moving to zone (ss)", 72, 0.7, 10, ss_strafeRight);
        Operation ss_turnToLine = new TurnOperation("Turning to line (ss)", 0, 0.65, 10 , ss_moveToZone);
        //endregion

        //region operations to drop the wobble in the QUADRUPLE STACK ZONE
        Operation qs_realign2 = new StrictTurnOperation("Realignment 2 (qs)", 0, 0.4, 10f, powerFlywheel);
        Operation qs_strafeLeft = new StrafeOperation("Towards wall (qs)", -6, 0.6, 10f, qs_realign2);
        Operation qs_toLaunch = new MoveOperation("To launch line (qs)", -64, 0.7, 10f, qs_strafeLeft);
        Operation qs_raiseWobble = new RaiseWobbleOperation(qs_toLaunch);
        Operation qs_lowerWobble = new LowerWobbleOperation(qs_raiseWobble);
        Operation qs_realign = new StrictTurnOperation("Realignment (qs)", 0, 0.25, 10f, qs_lowerWobble);
        Operation qs_strafeRight = new StrafeOperation("Away from wall (qs)", 10, 0.6, 10, qs_realign);
        Operation qs_moveToZone = new MoveOperation("Moving to zone (qs)", 106, 0.7, 10, qs_strafeRight);
        Operation qs_turnToLine = new TurnOperation("Turning to line (qs)", 0, 0.65, 10 , qs_moveToZone);
        //endregion

        Operation detect = new DetectOperation(2.5f, ns_turnToLine,
                                                                 ss_turnToLine,
                                                                 qs_turnToLine);
        Operation turnToRings = new TurnOperation("Turning to rings", 30, 0.65, 10, detect);
        Operation moveToRings = new MoveOperation("Moving to rings", 26, 0.6, 10, turnToRings);

        OperationRunner opRun = new OperationRunner(moveToRings);
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

            robot.printInformation(telemetry);

            telemetry.addData("Runner Status",opRun.getCurrentDisplay());

            telemetry.update();
        }
    }
}
