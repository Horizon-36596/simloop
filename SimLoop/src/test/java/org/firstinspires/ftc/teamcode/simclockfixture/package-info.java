/**
 * Test fixtures for SimLoop's sim-clock rewriting: small OpModes and helpers written the way an FTC team
 * writes them, with plain {@code ElapsedTime}, {@code System.nanoTime()}, {@code Thread.sleep} and
 * {@code sleep(ms)}, and no idea SimLoop exists.
 *
 * <p>They live under {@code org.firstinspires.ftc.teamcode.} on purpose, because that is the package
 * {@code SimClockLoader} rewrites by default. They are test sources only: SimLoop's main code never
 * references this package (its {@code SeparabilityTest} scans main sources only). Most report what they saw
 * through {@code OpMode.blackboard}, a map of the SDK's that the rewritten copy and the test share.
 */
package org.firstinspires.ftc.teamcode.simclockfixture;
