package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.Mecanum;

/**
 * Manual demo for FTC Relic Recovery game.
 */
@TeleOp(name="teleop", group="chargerbots")
public class RelicRecoveryManual extends RobotHardware {
    /**
     * Mecanum drive control program.
     */
    @Override
    public void loop() {
        super.loop();
        setDriveForMecanumForSpeed(Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x, gamepad1.right_stick_y));

        if (gamepad1.dpad_up) {
            raiseJewelArm();
        } else if (gamepad1.dpad_down) {
            lowerJewelArm();
        }

        if (gamepad1.dpad_right) {
            forwardJewelArm();
        } else if (gamepad1.dpad_left) {
            backwardJewelArm();
        } else {
            centerJewelArm();
        }
    }
}
