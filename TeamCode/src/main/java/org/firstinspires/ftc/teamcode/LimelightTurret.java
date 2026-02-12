package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class LimelightTurret extends OpMode {

    private PIDController controller;

    final int READ_PERIOD = 1;

    DcMotor intakeMotor;
    Servo pitchServo;
    DcMotor rotationMotor;
    DcMotor shootingMotor;
    DcMotor sorterMotor;
    Servo leverServo;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;
    Deadline rateLimit;

    Limelight3A limelight;

    public static double p = 0.03;
    public static double iS = 0;
    public static double d = 0.0002;
    public static double kFF = 0.02;

    IMU turretImu;

    // ADDED: encoder clip limits
    private static final int TURRET_ENCODER_HIGH_LIMIT = 1400;
    private static final int TURRET_ENCODER_LOW_LIMIT  = -1400;

    public int TAG_TARGET = 160;

    private static final int TARGET_TAG_ID = 20;

    double target = 0;

    double lastTx = 0;

    @Override
    public void init() {
        controller = new PIDController(p, iS, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretImu = hardwareMap.get(IMU.class, "turretImu");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pitchServo = hardwareMap.get(Servo.class,"pitchServo");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        shootingMotor = hardwareMap.get(DcMotor.class, "shootingMotor");
        sorterMotor = hardwareMap.get(DcMotor.class, "sorterMotor");
        leverServo = hardwareMap.get(Servo.class,"leverServo");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskylens2");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);


        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        leverServo.setPosition(0);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);


        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        turretImu.initialize(new IMU.Parameters(orientationOnRobot));
        turretImu.resetYaw();

        YawPitchRollAngles initAngles = turretImu.getRobotYawPitchRollAngles();

    }

    @Override
    public void loop() {


        /* All algorithms, except for LINE_TRACKING, return a list of Blocks where a
                * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
                * assign them to objects.
                *
             * Returns an empty array if no objects are seen.
             */


        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();


            lastTx = result.getTx();
        }

        controller.setPID(p, iS, d);
        double error = 0 - lastTx;

        double pidOutput = controller.calculate(lastTx, 0);
        double feedforward = Math.copySign(kFF, error);
        double motorPower = pidOutput + feedforward;

        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

        rotationMotor.setPower(motorPower);

        telemetry.addData("tx", lastTx);
        telemetry.addData("target", target);
        telemetry.addData("Error (deg)", error);
        telemetry.addData("ticks", rotationMotor.getCurrentPosition());
        telemetry.addData("Targets Visible", result != null ? result.getFiducialResults().size() : 0);
        telemetry.update();

            /*
             * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
             * - blocks[i].width and blocks[i].height   (size of box, in pixels)
             * - blocks[i].left and blocks[i].top       (edges of box)
             * - blocks[i].x and blocks[i].y            (center location)
             * - blocks[i].id                           (Color ID)
             *
             * These values have Java type int (integer).
             */


            telemetry.update();
        }
    }
