package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.0634603)
            .forwardZeroPowerAcceleration(-61.09176788994693)
            .lateralZeroPowerAcceleration(-95.02714753581012)
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.085, 0, 0.0032, 0.05))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.04, 0.02))
            //.headingPIDFCoefficients(new PIDFCoefficients(0.393, 0, 0.002, 0.045))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.005, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.035, 0, 0.001, 0.6, 0.025))
            .centripetalScaling(0.0006);


    public static PathConstraints pathConstraints = new PathConstraints
            (0.99, 100, 1, 1)
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build()

                ;
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(76.7842267854957)
            .yVelocity(48.24890617310532)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.156)
            .strafePodX(-2.75)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

}
