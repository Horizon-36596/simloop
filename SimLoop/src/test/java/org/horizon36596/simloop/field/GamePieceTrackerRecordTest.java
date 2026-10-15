package org.horizon36596.simloop.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.ScenarioRunner;

import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.rlog.RLOGReplay;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * INVARIANT-tier (domain R9) — what {@link GamePieceTracker#record(String)} actually puts in an RLOG,
 * decoded back out of a real one written by PsiKit through {@link ScenarioRunner}:
 *
 * <ol>
 *   <li>the counts and every positions key are written, the positions in <b>metres</b> (AdvantageScope's
 *       field space), z being the piece's centre height;</li>
 *   <li>a kind's {@code kinds/<kind>/positions3d} key is written on <b>every</b> tick, as an empty array
 *       once no piece of that kind is visible — otherwise the log keeps the last positions and
 *       AdvantageScope draws ghost pieces that were picked up long ago;</li>
 *   <li>a piece scored with the label-only {@code score} is in no positions key at all.</li>
 * </ol>
 *
 * <p>Writes into {@code build/sim} rather than a JUnit {@code @TempDir}, for the Windows file-handle reason
 * {@code ScenarioRunnerTest} gives.
 */
class GamePieceTrackerRecordTest {

    private static final double DT = 0.02;
    private static final double METRES_PER_INCH = 0.0254;
    private static final double RADIUS = 1.4;
    private static final String PREFIX = "GamePieces";

    /** A minimal robot config: ScenarioRunner records its class name as metadata and reads nothing else. */
    private static final SimRobotConfig STUB_CONFIG = new SimRobotConfig() {
        @Override public String[] driveMotorNames() { return new String[] {"fl", "fr", "bl", "br"}; }
        @Override public String odometryName() { return "odo"; }
        @Override public DrivetrainSimConfig drivetrain() {
            return new DrivetrainSimConfig() {
                @Override public double trackWidth() { return 12.0; }
                @Override public double wheelBase() { return 12.0; }
                @Override public double wheelRadius() { return 2.0; }
                @Override public double ticksPerInch() { return 100.0; }
                @Override public double maxVelocityTicksPerSecond() { return 1000.0; }
                @Override public double maxAccel() { return 2.0; }
                @Override public double fieldHalfWidth() { return 72.0; }
                @Override public double fieldHalfHeight() { return 72.0; }
            };
        }
    };

    @Test
    void recordWritesMetresEveryKindEveryTickAndLeavesOutPiecesThatLeftTheField() throws IOException {
        Path rlogPath = Paths.get("build", "sim", "game-piece-tracker-record-test.rlog");
        GamePieceTracker tracker = new GamePieceTracker(4);
        tracker.updateRobotPose(0.0, 0.0, 0.0);
        tracker.addLoosePiece("ball", 10.0, 20.0, RADIUS);          // id 0: stays loose all run
        tracker.addLoosePiece("cube", 8.0, 0.0, RADIUS);            // id 1: in the mouth, taken on tick 1
        tracker.addHeldPiece("ball", RADIUS);                       // id 2: scored label-only on tick 2
        TriggerVolume mouth = new TriggerVolume("intake", 8.0, 0.0, 6.0, 16.0);

        int[] tick = {0};
        ScenarioRunner.run("GamePieceTrackerRecordTest", STUB_CONFIG, new FakeTimer(), 4, DT, rlogPath,
                deltaTimeSeconds -> {
                    tracker.updateRobotPose(0.0, 0.0, 0.0);
                    if (tick[0] == 1) {
                        tracker.tryAcquire(mouth, true);            // the only cube goes into the robot
                    }
                    if (tick[0] == 2) {
                        tracker.score(2, "deposit");                // out of the field, into no goal
                    }
                    tracker.advance(deltaTimeSeconds);
                    tracker.record(PREFIX);
                    tick[0]++;
                });

        List<Map<String, LogTable.LogValue>> frames = decodeFrames(rlogPath);
        assertEquals(4, frames.size(), "one decoded frame per tick");

        // Tick 0: both kinds visible, in metres.
        double[] cubesAtStart = translations(frames.get(0), "kinds/cube/positions3d");
        assertEquals(3, cubesAtStart.length, "one cube on the field at the start");
        assertEquals(8.0 * METRES_PER_INCH, cubesAtStart[0], 1e-12, "x in metres, not inches");
        assertEquals(0.0, cubesAtStart[1], 1e-12);
        assertEquals(RADIUS * METRES_PER_INCH, cubesAtStart[2], 1e-12, "z is the centre height, in metres");
        double[] ballsAtStart = translations(frames.get(0), "kinds/ball/positions3d");
        assertEquals(3, ballsAtStart.length, "the held ball is not drawn; the loose one is");
        assertEquals(10.0 * METRES_PER_INCH, ballsAtStart[0], 1e-12);
        assertEquals(20.0 * METRES_PER_INCH, ballsAtStart[1], 1e-12);
        assertEquals(1L, frames.get(0).get(key("heldCount")).getInteger());
        assertEquals(2L, frames.get(0).get(key("looseCount")).getInteger());

        // Tick 1 onward: the cube is held. Its key must still be written, as an empty array; a key that is
        // simply not written keeps its old value in the log, and AdvantageScope keeps drawing the cube.
        for (int frame = 1; frame < 4; frame++) {
            assertEquals(0, translations(frames.get(frame), "kinds/cube/positions3d").length,
                    "frame " + frame + ": every cube is held, so the cube key is an empty array, not stale");
        }

        // Tick 2 onward: the label-only scored ball is counted but drawn nowhere.
        Map<String, LogTable.LogValue> last = frames.get(3);
        assertEquals(1L, last.get(key("scoredCount")).getInteger(), "counted as scored");
        assertEquals(0, translations(last, "scoredPositions3d").length, "but parked in no goal, so not drawn");
        double[] ballsAtEnd = translations(last, "kinds/ball/positions3d");
        assertEquals(3, ballsAtEnd.length, "only the loose ball is drawn, not the one scored into no goal");
        assertEquals(3, translations(last, "loosePositions3d").length);
        assertEquals(0, translations(last, "inFlightPositions3d").length);
    }

    private static String key(String suffix) {
        return "RealOutputs/" + PREFIX + "/" + suffix;
    }

    /**
     * A {@code Translation3d[]} key's raw struct bytes as a flat array {@code x0, y0, z0, x1, ...}: each
     * {@code Translation3d} is three little-endian doubles, the WPILib struct layout.
     */
    private static double[] translations(Map<String, LogTable.LogValue> frame, String suffix) {
        LogTable.LogValue value = frame.get(key(suffix));
        assertNotNull(value, "the log has no " + key(suffix) + " at all; keys present: " + frame.keySet());
        byte[] raw = value.getRaw();
        assertTrue(raw.length % 24 == 0, "a Translation3d is 24 bytes; got " + raw.length);
        ByteBuffer buffer = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN);
        double[] out = new double[raw.length / 8];
        for (int i = 0; i < out.length; i++) {
            out[i] = buffer.getDouble();
        }
        return out;
    }

    /** Every frame of the RLOG as the full logged state at that frame, the way AdvantageScope replays it. */
    private static List<Map<String, LogTable.LogValue>> decodeFrames(Path rlogPath) {
        RLOGReplay replay = new RLOGReplay(rlogPath.toString());
        replay.start();
        LogTable table = new LogTable(0);
        List<Map<String, LogTable.LogValue>> frames = new ArrayList<>();
        while (replay.updateTable(table)) {
            frames.add(new TreeMap<>(table.getAll(true)));
        }
        replay.end();
        return frames;
    }
}
