package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.changes;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@Configurable
@TeleOp(name = "Tuning", group = "Pedro Pathing")
public class Tuning extends SelectableOpMode {

    public static Follower follower;

    @IgnoreConfigurable
    public static PoseHistory poseHistory;

    @IgnoreConfigurable
    public static TelemetryManager telemetryM;

    @IgnoreConfigurable
    public static ArrayList<String> changes = new ArrayList<>();

    public Tuning() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Localization", l -> {
                l.add("Localization Test", LocalizationTest::new);
                l.add("Forward Tuner", ForwardTuner::new);
                l.add("Lateral Tuner", LateralTuner::new);
                l.add("Turn Tuner", TurnTuner::new);
            });
            s.folder("Automatic", a -> {
                a.add("Forward Velocity Tuner", ForwardVelocityTuner::new);
                a.add("Lateral Velocity Tuner", LateralVelocityTuner::new);
                a.add("Forward Zero Power Acceleration Tuner", ForwardZeroPowerAccelerationTuner::new);
                a.add("Lateral Zero Power Acceleration Tuner", LateralZeroPowerAccelerationTuner::new);
            });
            s.folder("Manual", p -> {
                p.add("Translational Tuner", TranslationalTuner::new);
                p.add("Heading Tuner", HeadingTuner::new);
                p.add("Drive Tuner", DriveTuner::new);
                p.add("Line Tuner", Line::new);
                p.add("Centripetal Tuner", CentripetalTuner::new);
            });
            s.folder("Tests", p -> {
                p.add("Line", Line::new);
                p.add("Triangle", Triangle::new);
                p.add("Circle", Circle::new);
            });
        });
    }

    @Override
    public void onSelect() {
        follower = Constants.createFollower(hardwareMap);
        PanelsConfigurables.INSTANCE.refreshClass(this);

        follower.setStartingPose(new Pose());
        poseHistory = follower.getPoseHistory();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Drawing.init();
    }

    @Override
    public void onLog(List<String> lines) {}

    public static void drawOnlyCurrent() {
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();
    }

    public static void draw() {
        Drawing.drawDebug(follower);
    }

    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
    }
}

/* ------------------------------
   ALL TUNER CLASSES BELOW
   ------------------------------ */

class LocalizationTest extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Localization test running.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.update(telemetry);

        draw();
    }
}

class ForwardTuner extends OpMode {
    public static double DISTANCE = 48;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Forward tuning.");
        telemetryM.update(telemetry);
        drawOnlyCurrent();
    }

    @Override
    public void loop() {
        follower.update();
        draw();
    }
}

class LateralTuner extends OpMode {
    public static double DISTANCE = 48;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Lateral tuning.");
        telemetryM.update(telemetry);
        drawOnlyCurrent();
    }

    @Override
    public void loop() {
        follower.update();
        draw();
    }
}

class TurnTuner extends OpMode {
    public static double ANGLE = 2 * Math.PI;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Turn tuning.");
        telemetryM.update(telemetry);
        drawOnlyCurrent();
    }

    @Override
    public void loop() {
        follower.update();
        draw();
    }
}

class ForwardVelocityTuner extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
    }

    @Override
    public void loop() {
        follower.update();
        draw();
    }
}

class LateralVelocityTuner extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
    }

    @Override
    public void loop() {
        follower.update();
        draw();
    }
}

class ForwardZeroPowerAccelerationTuner extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
    }

    @Override
    public void loop() {
        follower.update();
        draw();
    }
}

class LateralZeroPowerAccelerationTuner extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
    }

    @Override
    public void loop() {
        follower.update();
        draw();
    }
}

/* ------------------------------
   STUB CLASSES FOR MISSING TUNERS
   ------------------------------ */

class TranslationalTuner extends OpMode {
    @Override public void init() {}
    @Override public void loop() {}
}

class HeadingTuner extends OpMode {
    @Override public void init() {}
    @Override public void loop() {}
}

class DriveTuner extends OpMode {
    @Override public void init() {}
    @Override public void loop() {}
}

class Line extends OpMode {
    @Override public void init() {}
    @Override public void loop() {}
}

class Triangle extends OpMode {
    @Override public void init() {}
    @Override public void loop() {}
}

class Circle extends OpMode {
    @Override public void init() {}
    @Override public void loop() {}
}

class CentripetalTuner extends OpMode {
    @Override public void init() {}
    @Override public void loop() {}
}

/* ------------------------------
   DRAWING STUB (FIXES ERRORS)
   ------------------------------ */

class Drawing {
    public static void init() {}
    public static void drawRobot(Pose pose) {}
    public static void sendPacket() {}
    public static void drawDebug(Follower follower) {}
}
