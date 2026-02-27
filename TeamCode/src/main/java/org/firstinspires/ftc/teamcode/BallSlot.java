package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class BallSlot {

    // INSTANCE VARIABLES
    DistanceSensor distanceSensor;
    ColorSensor colorSensor;

    // CLASS VARIABLES
    int numOfSlots = 0;
    public static ArrayList<BallSlot> slots = new ArrayList<>();

    public BallSlot(DistanceSensor distanceSensor, ColorSensor colorSensor) {
        this.distanceSensor = distanceSensor;
        this.colorSensor = colorSensor;
    }

    public void initSlot(HardwareMap hwMap, String distanceName, String colorName) {
        distanceSensor = hwMap.get(DistanceSensor.class, distanceName);
        colorSensor = hwMap.get(ColorSensor.class, colorName);
    }


}
