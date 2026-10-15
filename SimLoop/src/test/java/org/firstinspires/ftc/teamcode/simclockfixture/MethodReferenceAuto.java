package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.LongConsumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/** Hands clocks and timers around as method references and lambdas, as a "clock" constructor argument does. */
@Autonomous(name = "Method References")
public class MethodReferenceAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        LongSupplier nanoClock = System::nanoTime;
        LongSupplier milliClock = System::currentTimeMillis;
        Supplier<ElapsedTime> timers = ElapsedTime::new;
        LongSupplier lambdaClock = () -> System.nanoTime();
        LongConsumer lambdaSleep = ms -> sleep(ms);

        waitForStart();
        long start = nanoClock.getAsLong();
        ElapsedTime timer = timers.get();
        lambdaSleep.accept(400);
        blackboard.put("nanoClock", nanoClock.getAsLong() - start);
        blackboard.put("lambdaClock", lambdaClock.getAsLong() - start);
        blackboard.put("milliClock", milliClock.getAsLong());
        blackboard.put("timer", timer.seconds());
        blackboard.put("timerClass", timer.getClass().getName());
        while (opModeIsActive()) {
            idle();
        }
    }
}
