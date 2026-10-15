package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.function.LongConsumer;

/** Passes this::sleep around, which SimLoop refuses: a method reference to sleep cannot be moved. */
@Autonomous(name = "Sleep Method Reference")
public class SleepMethodReferenceAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        LongConsumer pause = this::sleep;
        waitForStart();
        pause.accept(100);
    }
}
