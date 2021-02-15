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
import org.firstinspires.ftc.teamcode.operations.WaitOperation;

@Autonomous(name="Right Red Auto", group="Linear Opmode")
public class RR_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        OperationRunner opRun = new OperationRunner(new WaitOperation(5f, null));

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
