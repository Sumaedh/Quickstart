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

    public enum BallSlotStates {
        MANUAL,
        SHOOTING_LOGIC,
        AUTOSPINNER,
        COLOR_PICKER;
    }

    private BallSlotStates currentState;
    public void setBallSlotState(BallSlotStates newState) {
        currentState = newState;
    }

    private void autosortStateMachine() {
        switch (currentState) {
            case AUTOSPINNER:
                if (ballSlot2.hasBall() && sorter.sorterAtTarget()) {
                    for (int i= 0; i < 3; i++) {
                        // currentslot = ballSlots[i];
                        if (!ballSlots[i].hasBall() && i == 0) {
                            sorter.turnSorter(1);
                        } else if (!ballSlots[i].hasBall() && i == 2) {
                            sorter.target -= sorter.INCREMENT;
                        }
                    }
                }

                if ((sorter.sorterAtTarget() && ballSlot1.hasBall() && ballSlot2.hasBall() && ballSlot3.hasBall()) || gamepad1.x) {
                    setBallSlotState(BallSlotStates.SHOOTING_LOGIC);
                } else if (gamepad1.y) {
                    setBallSlotState(BallSlotStates.MANUAL);
                } else if (gamepad1.b) {
                    setBallSlotState(BallSlotStates.COLOR_PICKER);
                }
                break;
            case SHOOTING_LOGIC:
                // TODO: SHOOTING LOGIC HERE

                if ((sorter.sorterAtTarget() && !ballSlot1.hasBall() && !ballSlot2.hasBall() && !ballSlot3.hasBall()) || gamepad1.x) {
                    setBallSlotState(BallSlotStates.AUTOSPINNER);
                } else if (gamepad1.y) {
                    setBallSlotState(BallSlotStates.MANUAL);
                } else if (gamepad1.b) {
                    setBallSlotState(BallSlotStates.COLOR_PICKER);
                }
                break;
            case MANUAL:
                if (gamepad2.aWasPressed()) {
                    sorter.target += sorter.INCREMENT;
                }
                if (gamepad1.y) {
                    setBallSlotState(BallSlotStates.AUTOSPINNER);
                } else if (gamepad1.b) {
                    setBallSlotState(BallSlotStates.COLOR_PICKER);
                }
                break;
            case COLOR_PICKER:
                // GREEN IS RIGHT BUMPER, LEFT BUMPER FOR PURPLE
                if (gamepad1.rightBumperWasPressed() && sorter.sorterAtTarget()) {
                    for (int i = ballSlots.length - 1; i >= 0; i--) {
                        // currentslot = ballSlots[i];
                        if (ballSlots[i].detectColor().equals("green") && i == 2 && sorter.sorterAtTarget()) {
                            break;
                        }
                        else if (ballSlots[i].detectColor().equals("green") && i == 1 && sorter.sorterAtTarget()) {
                            sorter.turnSorter(1);
                        } else if (ballSlots[i].detectColor().equals("green") && i == 0 && sorter.sorterAtTarget()) {
                            sorter.target -= sorter.INCREMENT;
                        }
                    }
                }

                if (gamepad1.leftBumperWasPressed() && sorter.sorterAtTarget()) {
                    for (int i = ballSlots.length - 1; i >= 0; i--) {
                        // currentslot = ballSlots[i];
                        if (ballSlots[i].detectColor().equals("purple") && i == 1 && sorter.sorterAtTarget()) {
                            sorter.turnSorter(1);
                        } else if (ballSlots[i].detectColor().equals("purple") && i == 0 && sorter.sorterAtTarget()) {
                            sorter.target -= sorter.INCREMENT;
                        }
                    }
                }

                if (gamepad1.x) {
                    setBallSlotState(BallSlotStates.MANUAL);
                } else if (gamepad1.y) {
                    setBallSlotState(BallSlotStates.AUTOSPINNER);
                }
                break;

        }
    }


    @Override
    public void init() {
        ballSlot1.initSlot(hardwareMap, "distanceSensor1", "colorSensor1");
        ballSlot2.initSlot(hardwareMap, "distanceSensor2", "colorSensor2");
        ballSlot3.initSlot(hardwareMap, "distanceSensor3", "colorSensor3");

        sorter.initSorter(hardwareMap);
    }

    @Override
    public void start() {
        setBallSlotState(BallSlotStates.AUTOSPINNER);
    }

    @Override
    public void loop() {

        sorter.PIDFSorterLoop();

        for (int i= 0; i < 3; i++) {
            // currentslot = ballSlots[i];
            ballSlots[i].detectBall();
        }

        autosortStateMachine();

        /*

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

         */



    }
}
