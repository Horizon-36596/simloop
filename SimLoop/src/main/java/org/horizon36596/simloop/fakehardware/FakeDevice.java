package org.horizon36596.simloop.fakehardware;

/**
 * A simulated hardware device with a kinematic plant. The harness ticks every fake device once per
 * loop with the {@code FakeTimer}'s deltaTime. State mutation happens ONLY
 * in {@link #update(double)}; the SDK getters are pure reads. One exception:
 * {@code FakeGoBildaPinpoint} runs the real SDK driver, whose getter for any register outside the driver's
 * bulk-read scope reads the bus on every call, on every firmware - and the default scope already leaves out
 * the device ID, version, yaw scalar, pod offsets, quaternion, pitch and roll; its class comment says more.
 */
public interface FakeDevice {

    /**
     * Advance this device's kinematic state by {@code deltaTime} seconds.
     * Pure and deterministic — no wall-clock reads.
     *
     * @param deltaTime how far to advance, in seconds; the harness passes its fixed tick
     */
    void update(double deltaTime);
}
