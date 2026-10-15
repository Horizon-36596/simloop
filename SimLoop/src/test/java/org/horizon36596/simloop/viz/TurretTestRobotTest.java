package org.horizon36596.simloop.viz;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Pins {@code docs/briefs/turret-3d-viz.md} clauses 2 and 4 on SimLoop's turret test robot.
 *
 * <p>Clause 4 says every length comes from {@code mechanism.json}. The class is written by hand to mirror that file, so the
 * test reads the file itself (a test resource, found on the classpath) and checks the class says the same
 * numbers. It also checks every constant carries a provenance label ({@code CLAUDE.md} hard rule 3). The repo has
 * no JSON library, so the file is read with plain regular expressions, each scoped to one joint's own entry.
 *
 * <p>Invariant tier: both hold for any correct generation of this class.
 */
class TurretTestRobotTest {

    private static final String IR_RESOURCE = "turret-test-robot.mechanism.json";

    private static String readIr() throws IOException {
        InputStream in = TurretTestRobotTest.class.getResourceAsStream(IR_RESOURCE);
        assertNotNull(in, IR_RESOURCE + " is not on the test classpath");
        try (InputStream stream = in) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buffer = new byte[4096];
            for (int n; (n = stream.read(buffer)) > 0; ) {
                out.write(buffer, 0, n);
            }
            return new String(out.toByteArray(), StandardCharsets.UTF_8);
        }
    }

    /** One joint's own entry: from its {@code "name"} to its {@code "stateKey"}, so nothing leaks in from the next. */
    private static String jointScope(String ir, String joint) {
        int start = ir.indexOf("\"name\": \"" + joint + "\"");
        assertTrue(start >= 0, "the IR has no joint named " + joint);
        return ir.substring(start, ir.indexOf("\"stateKey\"", start));
    }

    private static double number(String scope, String key) {
        Matcher m = Pattern.compile("\"" + Pattern.quote(key) + "\":\\s*(-?[0-9.eE+-]+)").matcher(scope);
        assertTrue(m.find(), "no " + key + " in the IR");
        return Double.parseDouble(m.group(1));
    }

    private static RobotVector3 vector(String scope, String field) {
        int at = scope.indexOf("\"" + field + "\"");
        assertTrue(at >= 0, "no " + field + " in the IR");
        String object = scope.substring(at, scope.indexOf('}', at));
        return new RobotVector3(number(object, "x"), number(object, "y"), number(object, "z"));
    }

    private static void assertSameVector(RobotVector3 expected, RobotVector3 actual, String what) {
        assertEquals(expected.x, actual.x, 0.0, what + " x");
        assertEquals(expected.y, actual.y, 0.0, what + " y");
        assertEquals(expected.z, actual.z, 0.0, what + " z");
    }

    @Test
    void everyGeneratedNumberIsTheNumberInTheIr() throws IOException {
        String ir = readIr();
        TurretSketchDefinition turret = TurretTestRobot.create();
        RevoluteJointDefinition yaw = turret.getYaw();
        RevoluteJointDefinition hood = (RevoluteJointDefinition) turret.getJoints().get(0);
        String yawIr = jointScope(ir, "yaw");
        String hoodIr = jointScope(ir, "hood");

        assertSameVector(vector(ir, "mountPointInches"), turret.getYawPivotInches(), "mountPointInches");
        assertSameVector(vector(yawIr, "zeroPoseDirection"), yaw.getZeroPoseDirection(), "yaw zeroPoseDirection");
        assertSameVector(vector(yawIr, "rotationAxis"), yaw.getRotationAxis(), "yaw rotationAxis");
        assertEquals(number(yawIr, "armLengthInches"), yaw.getArmLengthInches(), 0.0);
        assertEquals(number(yawIr, "minAngleRadians"), yaw.getMinAngleRadians(), 0.0);
        assertEquals(number(yawIr, "maxAngleRadians"), yaw.getMaxAngleRadians(), 0.0);

        assertSameVector(vector(hoodIr, "zeroPoseDirection"), hood.getZeroPoseDirection(), "hood zeroPoseDirection");
        assertSameVector(vector(hoodIr, "rotationAxis"), hood.getRotationAxis(), "hood rotationAxis");
        assertEquals(number(hoodIr, "armLengthInches"), hood.getArmLengthInches(), 0.0);
        assertEquals(number(hoodIr, "minAngleRadians"), hood.getMinAngleRadians(), 0.0);
        assertEquals(number(hoodIr, "maxAngleRadians"), hood.getMaxAngleRadians(), 0.0);

        assertEquals(number(ir, "widthInches"), turret.getCanvasWidthInches(), 0.0);
        assertEquals(number(ir, "heightInches"), turret.getCanvasHeightInches(), 0.0);
    }

    /**
     * Each joint's {@code "stateKey"} in the JSON is the key the RLOG test logs that joint under
     * ({@link TurretTestRobot#YAW_STATE_KEY}, {@link TurretTestRobot#HOOD_STATE_KEY}), so the fixture and the
     * log cannot drift apart.
     */
    @Test
    void everyStateKeyInTheIrIsTheKeyTheTestRobotLogs() throws IOException {
        String ir = readIr();
        assertEquals(TurretTestRobot.YAW_STATE_KEY, stateKey(ir, "yaw"), "yaw stateKey");
        assertEquals(TurretTestRobot.HOOD_STATE_KEY, stateKey(ir, "hood"), "hood stateKey");
    }

    /** The {@code "stateKey"} string of one joint's entry. */
    private static String stateKey(String ir, String joint) {
        int start = ir.indexOf("\"name\": \"" + joint + "\"");
        assertTrue(start >= 0, "the IR has no joint named " + joint);
        Matcher m = Pattern.compile("\"stateKey\":\\s*\"([^\"]*)\"").matcher(ir);
        assertTrue(m.find(start), "no stateKey for joint " + joint);
        return m.group(1);
    }

    /** Clause 4's other half: every numeric constant in the generated class carries a provenance label. */
    @Test
    void everyGeneratedConstantCarriesAProvenanceLabel() throws IOException {
        String source = readSource();
        Matcher constant = Pattern.compile(
                "/\\*\\*((?:(?!\\*/).)*)\\*/\\s*(?:private )?static final (double|RobotVector3) (\\w+)",
                Pattern.DOTALL).matcher(source);
        int constants = 0;
        while (constant.find()) {
            constants++;
            assertTrue(constant.group(1).matches(
                            "(?s).*\\b(MEASURED|SET BY HAND|SPEC|NEEDS MEASUREMENT|CARRIED OVER|PLACEHOLDER)\\b.*"),
                    constant.group(3) + " has no provenance label in its javadoc");
        }
        assertTrue(constants >= 10, "found only " + constants + " constants; did the pattern stop matching?");
    }

    private static String readSource() throws IOException {
        String relative = "src/test/java/org/horizon36596/simloop/viz/TurretTestRobot.java";
        for (String candidate : new String[] {relative, "SimLoop/" + relative}) {
            Path path = Paths.get(candidate);
            if (Files.exists(path)) {
                return new String(Files.readAllBytes(path), StandardCharsets.UTF_8);
            }
        }
        throw new IOException("TurretTestRobot.java not found from " + Paths.get("").toAbsolutePath());
    }

    /** Clause 2: the hood is driven by its angle in degrees from horizontal, and drawn at exactly that. */
    @Test
    void theHoodIsDrawnAtTheAngleAShooterStatesInDegreesFromHorizontal() {
        TurretSketch sketch = new TurretSketch(TurretTestRobot.create());
        for (double hoodDeg = 30.0; hoodDeg <= 70.0; hoodDeg += 5.0) {
            sketch.setJointAngleRadians(TurretTestRobot.HOOD_JOINT, TurretTestRobot.hoodJointRadiansFor(hoodDeg));
            assertEquals(Math.toRadians(hoodDeg), sketch.getAimElevationRadians(), 1.0e-9,
                    "a " + hoodDeg + " degree hood must aim " + hoodDeg + " degrees above horizontal");
        }
    }

    /** The yaw arm is vertical, so the turret faces the way its hood swings: out of the nose. */
    @Test
    void atZeroYawTheTestRobotsTurretFacesOutOfTheNose() {
        TurretSketch sketch = new TurretSketch(TurretTestRobot.create());
        assertEquals(0.0, sketch.getAimHeadingRadians(), 1.0e-12);
        assertEquals(-2.0 + 5.0 * Math.cos(Math.toRadians(30.0)), sketch.getAimFieldXInches(), 1.0e-9,
                "the exit is the hood's length ahead of the axis, which is 2 in behind the robot's centre");
        assertEquals(8.0 + 6.0 + 5.0 * Math.sin(Math.toRadians(30.0)), sketch.getAimFieldZInches(), 1.0e-9);
    }
}
