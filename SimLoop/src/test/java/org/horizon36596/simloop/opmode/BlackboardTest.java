package org.horizon36596.simloop.opmode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/**
 * Pins how a run treats the SDK's static {@code OpMode.blackboard}: empty at INIT by default, so one test's
 * run cannot leak into the next; kept on request, so an auto can hand its end pose to the TeleOp after it the
 * way it does on a robot; and never cleared after a run, so a test can read what the OpMode left.
 *
 * <p>Invariant tier: a test's outcome must not depend on which tests ran before it in the same JVM.
 */
class BlackboardTest {

    private static final String END_HEADING = "endHeadingDegrees";

    @AfterEach
    void leaveTheBlackboardAsWeFoundIt() {
        OpMode.blackboard.clear();
    }

    /** An auto that records where it finished, the SDK's ConceptBlackboard way. */
    static final class Auto extends LinearOpMode {
        @Override
        public void runOpMode() {
            waitForStart();
            blackboard.put(END_HEADING, 90.0);
        }
    }

    /** A TeleOp that reads the auto's end heading in init, and counts how often it has been started. */
    static final class TeleOp extends OpMode {
        Object headingSeenInInit;
        Object timesStartedSeen;

        @Override
        public void init() {
            headingSeenInInit = blackboard.get(END_HEADING);
            int times = (int) blackboard.getOrDefault("timesStarted", 0);
            blackboard.put("timesStarted", times + 1);
            timesStartedSeen = times + 1;
        }

        @Override
        public void loop() {
        }
    }

    private static OpModeRunner runner(OpMode opMode) {
        return new OpModeRunner(opMode, new FakeHardwareMap()).activeTicks(2);
    }

    @Test
    void aRunStartsWithAnEmptyBlackboardByDefault() {
        OpMode.blackboard.put(END_HEADING, "left over from another test");
        TeleOp teleOp = new TeleOp();
        runner(teleOp).run();
        assertNull(teleOp.headingSeenInInit);
        assertEquals(1, teleOp.timesStartedSeen);
    }

    @Test
    void whatTheOpModeLeftIsStillThereAfterTheRunForTheTestToRead() {
        runner(new Auto()).run();
        assertEquals(90.0, OpMode.blackboard.get(END_HEADING));
    }

    @Test
    void keepBlackboardHandsTheAutosDataToTheTeleOpAfterIt() {
        runner(new Auto()).run();
        TeleOp teleOp = new TeleOp();
        runner(teleOp).keepBlackboard(true).run();
        assertEquals(90.0, teleOp.headingSeenInInit);
    }

    @Test
    void withoutKeepBlackboardTheTeleOpAfterTheAutoSeesNothing() {
        runner(new Auto()).run();
        TeleOp teleOp = new TeleOp();
        runner(teleOp).run();
        assertNull(teleOp.headingSeenInInit);
    }

    @Test
    void keptAcrossManyRunsItAccumulatesAsOnARobotAndOneDefaultRunResetsIt() {
        for (int run = 1; run <= 3; run++) {
            TeleOp teleOp = new TeleOp();
            runner(teleOp).keepBlackboard(run > 1).run();
            assertEquals(run, teleOp.timesStartedSeen, "run " + run);
        }
        TeleOp fresh = new TeleOp();
        runner(fresh).keepBlackboard(false).run();
        assertEquals(1, fresh.timesStartedSeen);
    }
}
