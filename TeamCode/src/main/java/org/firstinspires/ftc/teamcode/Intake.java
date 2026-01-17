package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Intake {

    DcMotor intakeMotor;

    public void initIntake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intakeOn() {
        intakeMotor.setPower(-1);
    }

    public void intakeOff() {
        intakeMotor.setPower(1);
    }
}
