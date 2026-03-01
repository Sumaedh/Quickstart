package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.io.CharArrayWriter;

@Config
@TeleOp
public class BallSlotTest extends OpMode {

    DistanceSensor distanceSensor1;
    ColorSensor colorSensor1;

    DistanceSensor distanceSensor2;
    ColorSensor colorSensor2;

    DistanceSensor distanceSensor3;
    ColorSensor colorSensor3;

    BallSlot ballSlot1 = new BallSlot(distanceSensor1, colorSensor1);
    BallSlot ballSlot2 = new BallSlot(distanceSensor2, colorSensor2);
    BallSlot ballSlot3 = new BallSlot(distanceSensor3, colorSensor3);

    Sorter sorter = new Sorter();

    BallSlot[] ballSlots = {ballSlot1, ballSlot2, ballSlot3};


    @Override
    public void init() {
        ballSlot1.initSlot(hardwareMap, "distanceSensor1", "colorSensor1");
        ballSlot2.initSlot(hardwareMap, "distanceSensor2", "colorSensor2");
        ballSlot3.initSlot(hardwareMap, "distanceSensor3", "colorSensor3");

        sorter.initSorter(hardwareMap);
    }

    @Override
    public void loop() {

        sorter.PIDFSorterLoop();

        for (int i= 0; i < 3; i++) {
            // currentslot = ballSlots[i];
            ballSlots[i].detectBall();
        }


        if (ballSlot2.hasBall()) {
            for (int i= 0; i < 3; i++) {
                // currentslot = ballSlots[i];
                if (!ballSlots[i].hasBall() && i == 0) {
                    sorter.turnSorter(1);
                } else if (!ballSlots[i].hasBall() && i == 2) {
                    sorter.target -= sorter.INCREMENT;
                }
            }
        }

        // GREEN FOR A, B FOR PURPLE
        if (gamepad1.aWasPressed()) {
            for (int i = ballSlots.length - 1; i <= 0; i--) {
                // currentslot = ballSlots[i];
                if (ballSlots[i].detectColor() == "green" && i == 2) {
                    sorter.turnSorter(1);
                } else if (!ballSlots[i].hasBall() && i == 2) {
                    sorter.target -= sorter.INCREMENT;
                }
            }
        }

    }
}
