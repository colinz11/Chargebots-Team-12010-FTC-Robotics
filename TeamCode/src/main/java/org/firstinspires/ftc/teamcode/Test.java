package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/*
Created By Colin Zhu
 */

@Autonomous(name="Test", group ="test")
public class Test extends LinearOpMode {

    private RobotAutonomous robot = new RobotAutonomous();



    @Override public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        robot.gyro.calibrate();

        while (!isStopRequested() && robot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.update();

        waitForStart();

        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
            telemetry.update();
        }
        robot.encoderDrive(0.2,10,10,10);
    }
}
