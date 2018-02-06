package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Created By Colin Zhu
 */

@TeleOp(name="LETS WIN", group="TeleOp")
public class TeleOpMain extends OpMode
{
    private Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    private static final double liftPower = 1.0;
    private static final double relicPower = 0.8;
    private double direction = 1;

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){
        runtime.reset();
    }

    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y; // push joystick1 forward to go forward
        double right = gamepad1.left_stick_x; // push joystick1 to the right to strafe right
        double clockwise = gamepad1.right_stick_x; // push joystick2 to the right to rotate clockwise

        //Mechanum Drive
        robot.mechanumDrive(forward, clockwise, right);

        //Lift
        if(gamepad2.left_bumper)
            robot.motorLift.setPower(liftPower);
        else if(gamepad2.right_bumper)
            robot.motorLift.setPower(-liftPower);
        else
            robot.motorLift.setPower(0);


        //Glyph Claw
        robot.leftClaw.setPosition((robot.leftClaw.getPosition() + gamepad2.left_trigger * 0.01));
        robot.rightClaw.setPosition(robot.rightClaw.getPosition() - gamepad2.left_trigger * 0.01);

        robot.leftClaw.setPosition((robot.leftClaw.getPosition() - gamepad2.right_trigger * 0.01));
        robot.rightClaw.setPosition((robot.rightClaw.getPosition() + gamepad2.right_trigger * 0.01));

        //Jewel arm
        if(gamepad1.dpad_up){
            robot.jewel.setPosition(0);
        }

        //Relic Arm
        if(gamepad1.a){
            robot.relicArm.setPower(relicPower);
        }
        else if(gamepad1.b){
            robot.relicArm.setPower(-relicPower);
        }
        else{
            robot.relicArm.setPower(0);
        }

        //Relic Wrist
        robot.relicWrist.setPosition(robot.relicWrist.getPosition() + gamepad1.right_trigger * 0.0001);
        robot.relicWrist.setPosition(robot.relicWrist.getPosition() - gamepad1.left_trigger * 0.0001);


        //Relic Claw
        if(gamepad1.right_bumper){
            robot.relicClaw.setPosition(0);
        }

        if(gamepad1.left_bumper){
            robot.relicClaw.setPosition(1);
        }

        telemetry.addData("Power", robot.relicWrist.getPosition());
        telemetry.update();
    }

}
