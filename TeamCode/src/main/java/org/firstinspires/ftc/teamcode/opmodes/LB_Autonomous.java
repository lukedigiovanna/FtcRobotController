package org.firstinspires.ftc.teamcode.opmodes;

import android.app.job.JobInfo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.operations.MoveOperation;
import org.firstinspires.ftc.teamcode.operations.Operation;
import org.firstinspires.ftc.teamcode.operations.OperationRunner;
import org.firstinspires.ftc.teamcode.operations.StrafeOperation;
import org.firstinspires.ftc.teamcode.operations.TurnOperation;

@Autonomous(name="Left Blue Auto", group="Linear Opmode")
public class LB_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //region Detect dropzone
    Operation turnToReadRings;
    //endregion

    //region Moving firing position
    Operation
    moveToShoot,
    strafeToShoot;
    //endregion

    //region Shooting rings
    Operation
    setFlywheelPower,
    shootRing1,
    bringBumperBack1,
    shootRing2,
    bringBumperBack2,
    runRollers,
    stopRollers,
    shootRing3,
    bringBumperBack3,
    stopFlywheel;
    //endregion

    //region Closest dropzone
    Operation strafeLeftToDropzone;
    //endregion

    //region Mid dropzone
    Operation moveToZone;
    //endregion

    //region Far dropzone
    Operation
    turnToFaceZone,
    driveToZone;
    //endregion

    //region Drop wobble
    Operation
    dropWobble,
    raiseWobble,
    retractWobble;
    //endregion

    //region Parking
    Operation parkOnLineFarZone;
    Operation parkOnLineMidZone;
    //endregion

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);
        Operation.setRobot(robot);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        moveToShoot = new MoveOperation("Moving to firing position", 60, 0.5, 10, strafeToShoot);

        OperationRunner opRun = new OperationRunner(moveToShoot);
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
