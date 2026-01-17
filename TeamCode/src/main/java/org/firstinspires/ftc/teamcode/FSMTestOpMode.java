package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class FSMTestOpMode extends OpMode {

    private Shooter shooter = new Shooter();
    private Sorter sorter = new Sorter();
    private Lever lever = new Lever();

    private ShooterSorterLeverFSM fsm;

    // ensure we only press the virtual button once at the first loop
    private boolean startPressedOnce = false;
    private boolean firstLoopAfterStart = true;

    @Override
    public void init() {
        // initialize the provided subsystem objects (they manage hardware)
        shooter.initShooter(hardwareMap);
        sorter.initSorter(hardwareMap);
        lever.initLever(hardwareMap);

        fsm = new ShooterSorterLeverFSM(shooter, sorter, lever);

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // mark that we want to press the 'start sequence' button on the very first loop invocation
        startPressedOnce = true;
    }

    @Override
    public void loop() {
        double now = System.currentTimeMillis() / 1000.0;

        boolean seqButtonThisLoop = false;
        if (startPressedOnce) {
            // press once
            seqButtonThisLoop = true;
            startPressedOnce = false; // ensure only one rising edge
        }

        // run the FSM (it internally calls shooter.PIDFShootingLoop() and sorter.PIDFSorterLoop())
        fsm.run(now, seqButtonThisLoop);

        // Telemetry so you can watch progress on the driver station
        telemetry.addData("FSM running", fsm.isRunning());
        telemetry.addData("FSM stage", fsm.getStage());
        telemetry.addData("Shooter target", shooter.curTargetVelocity);
        telemetry.addData("Sorter target ticks", sorter.target);
        telemetry.addData("Sorter at target", sorter.SorterAtTarget());
        telemetry.update();
    }

    @Override
    public void stop() {
        // ensure motors stop
        // DO NOT call subsystem PID loops here; just zero critical outputs if desired
    }
}
