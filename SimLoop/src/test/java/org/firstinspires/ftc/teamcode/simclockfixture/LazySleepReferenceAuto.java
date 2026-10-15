package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.function.LongConsumer;

/**
 * A this::sleep-style method reference in a helper class first used after PLAY, inside a catch-all: the
 * refusal comes mid-run, as an exception the team's code catches.
 */
@Autonomous(name = "Lazy Sleep Reference")
public class LazySleepReferenceAuto extends LinearOpMode {

    /** Holds a bound method reference to the OpMode's sleep, which SimLoop cannot move. */
    static final class Pauser {
        private final LongConsumer pause;

        Pauser(LinearOpMode opMode) {
            pause = opMode::sleep;
        }

        void pause(long ms) {
            pause.accept(ms);
        }
    }

    @Override
    public void runOpMode() {
        waitForStart();
        sleep(100);
        try {
            new Pauser(this).pause(500);
            blackboard.put("helper", "ran");
        } catch (RuntimeException | LinkageError swallowed) {
            blackboard.put("helper", "caught " + swallowed.getClass().getSimpleName());
        }
        while (opModeIsActive()) {
            idle();
        }
    }
}
