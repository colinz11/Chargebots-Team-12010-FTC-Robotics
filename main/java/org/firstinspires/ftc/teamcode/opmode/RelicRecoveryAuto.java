package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.base.BlackBox;
import org.firstinspires.ftc.teamcode.base.Color;
import org.firstinspires.ftc.teamcode.base.StateMachine;
import org.firstinspires.ftc.teamcode.base.StateMachine.State;

import java.io.FileInputStream;

/**
 * Autonomous demo for FTC Relic Recovery game.
 * Assumes Jewel arm and the phone's camera are mounted on right of robot,
 * which influences expected orientations at start.
 */
public class RelicRecoveryAuto extends RobotHardware {

    @Autonomous(name="cb.Red.Center", group="chargerbots")
    public static class RedCenter extends org.firstinspires.ftc.teamcode.opmode.RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="cb.Red.Corner", group="chargerbots")
    public static class RedCorner extends org.firstinspires.ftc.teamcode.opmode.RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Autonomous(name="cb.Blue.Center", group="chargerbots")
    public static class BlueCenter extends org.firstinspires.ftc.teamcode.opmode.RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="cb.Blue.Corner", group="chargerbots")
    public static class BlueCorner extends org.firstinspires.ftc.teamcode.opmode.RelicRecoveryAuto {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();

        telemetry.addData("Robot Color", robotColor.name());
        telemetry.addData("Robot Start Position", robotStartPos.name());

        State playback = new BlackboxPlayback(
                getStartPositionName(robotColor, robotStartPos), null);
        State hitJewel = newHitJewel(playback);
        machine = new StateMachine(hitJewel);

        telemetry.update();
    }

    @Override
    public void loop() {
        super.loop();
        machine.update();
        telemetry.update();
    }

    // State in the machine to wait for a duration.
    private class WaitForDuration implements State {
        public WaitForDuration(double duration, State next) {
            this.duration = duration;
            this.next = next;
        }

        @Override
        public void start() {
            startTime = time;
        }

        @Override
        public State update() {
            double elapsed = time - startTime;
            telemetry.addData("Elapsed", elapsed);
            telemetry.addData("Duration", duration);
            if (elapsed > duration) {
                return next;
            }
            return this;
        }

        private double duration;
        private State next;
        private double startTime;
    }

    // Drops the jewel arm.
    private class DropJewelArm implements State {
        public DropJewelArm(State next) {
            this.next = next;
        }

        @Override
        public void start() {}

        @Override
        public State update() {
            setColorSensorLedEnabled(ColorSensorName.JEWEL, true);
            lowerJewelArm();
            return new WaitForDuration(2, next);
        }

        private State next;
    }

    // Reads the jewel color.
    private class HitJewel implements State {
        public HitJewel(State next) {
            this.next = next;
        }

        @Override
        public void start() {}

        @Override
        public State update() {
            int r = getColorSensor(ColorSensorName.JEWEL,
                                   Color.Channel.RED);
            int b = getColorSensor(ColorSensorName.JEWEL,
                                   Color.Channel.BLUE);

            if ((r > b && robotColor == Color.Ftc.BLUE) ||
                    (b > r && robotColor == Color.Ftc.RED)) {
                // Reading other team's jewel in forward position.
                forwardJewelArm();
            } else {
                // Reading our team's jewel in forward position.
                backwardJewelArm();
            }
            setColorSensorLedEnabled(ColorSensorName.JEWEL, false);
            return new WaitForDuration(1, next);
        }

        private State next;
    }

    // Resets the jewel arm to the starting position.
    private class ResetJewelArm implements State {
        public ResetJewelArm(State next) {
            this.next = next;
        }

        @Override
        public void start() {}

        @Override
        public State update() {
            raiseJewelArm();
            centerJewelArm();
            return new WaitForDuration(1, next);
        }

        private State next;
    }

    private State newHitJewel(State next) {
        State jewelReset = new ResetJewelArm(next);
        State jewelHit = new HitJewel(jewelReset);
        State jewelDrop = new DropJewelArm(jewelHit);
        return jewelDrop;
    }

    // Plays back a recorded blackbox stream.
    private class BlackboxPlayback implements State {
        public BlackboxPlayback(String filename, State next) {
            try {
                this.filename = filename;
                inputStream = hardwareMap.appContext.openFileInput(filename);
                player = new BlackBox.Player(inputStream, hardwareMap);
            } catch (Exception e) {
                e.printStackTrace();
                requestOpModeStop();
            }
            this.next = next;
        }

        @Override
        public void start() {
            startTime = time;
        }

        @Override
        public State update() {
            telemetry.addData("Playing File", filename);
            double elapsed = time - startTime;
            telemetry.addData("Elapsed", elapsed);
            try {
                if (player.playback(elapsed)) {
                    return this;
                } else {
                    return next;
                }
            } catch (Exception e) {
                e.printStackTrace();
                requestOpModeStop();
                return null;
            }
        }

        // The filename to playback.
        private String filename;
        // The next state after playback.
        private State next;
        // The input file stream.
        private FileInputStream inputStream;
        // The hardware player.
        private BlackBox.Player player;
        // The start time of playback.
        private double startTime;
    }

    // The robot's color.
    protected Color.Ftc robotColor;
    // The robot's starting position.
    protected StartPosition robotStartPos;

    // The state machine.
    private StateMachine machine;
}
