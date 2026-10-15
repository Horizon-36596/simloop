package org.horizon36596.simloop.fakehardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.junit.jupiter.api.Test;

/**
 * Robot code asks the hardware map for a {@code GoBildaPinpointDriver} by name and gets the fake; the map's
 * tick runs the fake's device clock. INVARIANT tier: that is how a SimLoop user wires it in.
 */
class FakeGoBildaPinpointHardwareMapTest {

    @Test
    void invariant_robotCodeGetsTheFakeAsTheSdkDriverAndTheMapTicksIt() {
        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        FakeGoBildaPinpoint fake = new FakeGoBildaPinpoint();
        hardwareMap.register("pinpoint", fake);

        GoBildaPinpointDriver odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        fake.moveChassisTo(40.0, 0.0, 0.0);
        hardwareMap.updateAll(0.02);
        odometry.update();

        assertSame(fake, odometry);
        assertEquals(40.0, odometry.getPosX(DistanceUnit.MM), 1e-4);
        assertEquals(40.0 / 0.02, odometry.getVelX(DistanceUnit.MM), 1e-2, "the map's tick is the device's clock");
    }
}
