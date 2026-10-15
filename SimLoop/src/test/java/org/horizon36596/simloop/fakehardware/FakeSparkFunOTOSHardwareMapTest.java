package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.junit.jupiter.api.Test;

/**
 * Robot code asks the hardware map for a {@code SparkFunOTOS} by name and gets the fake, already initialised
 * the way the SDK's map initialises one; the map's tick runs the fake's clock. INVARIANT tier: that is how a
 * SimLoop user wires it in.
 */
class FakeSparkFunOTOSHardwareMapTest {

    @Test
    void invariant_robotCodeGetsTheFakeAsTheSdkDriverAndTheMapTicksIt() {
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        FakeSparkFunOTOS fake = new FakeSparkFunOTOS();
        hardwareMap.register("otos", fake);

        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, " otos ");
        assertSame(fake, otos, "the SDK's name trimming, and the SDK type robot code asks for");
        assertTrue(hardwareMap.getAll(SparkFunOTOS.class).contains(fake));

        // What the SDK's SensorSparkFunOTOS sample does in init.
        otos.setLinearUnit(DistanceUnit.METER);
        otos.setOffset(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
        assertTrue(otos.setLinearScalar(1.0));
        assertTrue(otos.setAngularScalar(1.0));
        assertTrue(otos.calibrateImu());
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));

        fake.moveChassisTo(40.0, 0.0, 0.0);
        hardwareMap.updateAll(0.02);

        assertEquals(0.04, otos.getPosition().x, 1.0 / 3276.8);
        assertEquals(0.04 / 0.02, otos.getVelocity().x, 1.0 / 6553.6, "the map's tick is the sensor's clock");
    }
}
