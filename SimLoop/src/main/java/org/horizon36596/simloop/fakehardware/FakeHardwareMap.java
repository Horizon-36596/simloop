package org.horizon36596.simloop.fakehardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.SerialNumber;

import java.util.ArrayList;
import java.util.List;

/**
 * JVM-constructible {@link HardwareMap} that resolves fake devices by config name. Devices are registered
 * under the SAME names the real robot config uses, so the same subsystem code resolves them in sim and on
 * the robot.
 *
 * <p>Subclassing the SDK {@code HardwareMap} with a null context is the JVM seam PsiKit's own test
 * {@code JVMHardwareMap} relies on. The harness ticks the plant via {@link #updateAll(double)}.
 *
 * <p><b>Registration goes through the SDK's own storage, not a map of our own.</b> This class used to keep
 * a private {@code Map<String, HardwareDevice>} and override {@code get} to read it. That made the fake map
 * detectable by any robot code that used the rest of the {@link HardwareMap} surface: {@code tryGet} returned
 * null, {@code getAll(DcMotor.class)} came back empty, iterating the map yielded nothing, {@code size()} was
 * 0, and the typed mappings such as {@code hardwareMap.dcMotor} held nothing — all while {@code get} worked.
 * A private map also silently dropped the SDK's own naming rules: it did not trim the name, and registering
 * two devices of different types under one name overwrote the first instead of keeping both. Registering
 * through {@link DeviceMapping#put} gets every one of those behaviours from the SDK rather than
 * re-implementing them, which is what the fidelity rule asks for.
 *
 * <h2>Registering the same name or the same device twice</h2>
 * These are the SDK's rules, because registration goes through the SDK's own {@code DeviceMapping.put}:
 * <ul>
 *   <li>The same device again, under the same or another name: one device, known by each name.</li>
 *   <li>A different device of the <b>same</b> type under a name already taken: the new one
 *       <b>replaces</b> the old — {@code DeviceMapping.put} removes whatever that mapping held under the
 *       name before adding the new device.</li>
 *   <li>A device of a <b>different</b> type under a taken name: both are kept, and a typed lookup returns
 *       the one that fits.</li>
 * </ul>
 * {@link #updateAll(double)} ticks exactly the fakes the map currently holds, once each: a device
 * registered under three names is ticked once, and a device that has been replaced or removed is no longer
 * part of the simulated robot and is not ticked at all.
 *
 * <h2>One deliberate difference: enumeration order</h2>
 * The SDK builds the list behind {@code getAll}, {@code size} and iteration out of a {@code HashSet} of
 * devices, and a device that does not override {@code hashCode} hashes by identity — so on a real robot
 * {@code getAll(DcMotor.class)} comes back in an order that can change from one run to the next. A sim that
 * changed order between runs could not replay deterministically, so this map keeps that list in
 * <b>registration order</b> instead. The set of devices returned is exactly the SDK's; only the order is
 * pinned. Robot code that depends on the SDK's order is depending on something the robot does not promise.
 */
public class FakeHardwareMap extends HardwareMap {

    /**
     * Every fake to tick, once each, in the order it was first registered, so {@link #updateAll(double)}
     * advances plants in a fixed sequence. Holds exactly the registered fakes still in the map (class
     * javadoc). This list is never read as a lookup table — that is what the inherited map is for.
     */
    private final List<FakeDevice> devicesToTickInRegistrationOrder = new ArrayList<>();

    /**
     * Every device currently in the inherited map, once each, in the order it was first put there. Copied
     * into the SDK's {@code allDevicesList} after each change so {@code getAll}, {@code size} and iteration
     * enumerate in this order rather than in identity-hash order (see the class javadoc).
     */
    private final List<HardwareDevice> allDevicesInFirstPutOrder = new ArrayList<>();

    /** Creates an empty map; register each fake device with {@link #register(String, HardwareDevice)}. */
    public FakeHardwareMap() {
        super(null, null);
    }

    /**
     * Registers a fake device (which is both an SDK {@link HardwareDevice} and a {@link FakeDevice}) under
     * {@code name}, exactly as the real robot-configuration parser registers the hardware it replaces.
     *
     * @param <T>    the device's type, which must be both an SDK device and a tickable fake
     * @param name   the config name the robot code will look the device up by, matching the robot's real
     *               configuration file; unitless, and leading and trailing whitespace is trimmed by the
     *               SDK exactly as it is on a real Control Hub
     * @param device the fake to return under that name, and to tick on {@link #updateAll(double)}. A device
     *               already registered (under this or any other name) is still ticked only once; a device
     *               of the same type that this one replaces under {@code name} stops being ticked.
     */
    public <T extends HardwareDevice & FakeDevice> void register(String name, T device) {
        // The same device under the same name again changes nothing in the SDK's end state. Returning early
        // matters for order: DeviceMapping.put would remove and re-add it, moving it to the back of both the
        // tick order and the enumeration order.
        List<HardwareDevice> alreadyUnderThatName = allDevicesMap.get(name.trim());
        if (alreadyUnderThatName != null && containsThisExactObject(alreadyUnderThatName, device)) {
            return;
        }
        putIntoTheMatchingTypedMapping(name, device);
        if (!containsThisExactObject(devicesToTickInRegistrationOrder, device)) {
            devicesToTickInRegistrationOrder.add(device);
        }
        stopTickingDevicesNoLongerInTheMap();
    }

    /**
     * Drops from the tick list every fake the SDK no longer holds under any name — one replaced by a later
     * registration, or removed — so {@link #updateAll(double)} never advances a device robot code cannot
     * reach.
     */
    private void stopTickingDevicesNoLongerInTheMap() {
        for (int i = devicesToTickInRegistrationOrder.size() - 1; i >= 0; i--) {
            if (!isRegisteredUnderAnyName(devicesToTickInRegistrationOrder.get(i))) {
                devicesToTickInRegistrationOrder.remove(i);
            }
        }
    }

    /**
     * Files the device into the type-specific {@link DeviceMapping} the real SDK would have used, so
     * {@code hardwareMap.dcMotor}, {@code hardwareMap.servo} and friends contain what a robot expects.
     * {@code DeviceMapping.put} also writes the overall map, so one call covers both.
     *
     * <p>The chain is written out device type by device type rather than derived by reflection because a
     * reader has to be able to see which mapping a fake lands in without running it. A device whose type
     * has no mapping here — {@code FakeI2cDeviceSynchSimple} is the current example, since the SDK's
     * {@code i2cDeviceSynch} mapping is typed to the richer {@code I2cDeviceSynch} — still goes into the
     * overall map, which is exactly what the SDK's own {@link HardwareMap#put(String, HardwareDevice)}
     * is for.
     *
     * <p><b>First match wins, and one case relies on that.</b> {@code FakeExternalEncoder} implements
     * {@code DcMotorEx}, so it takes the {@code DcMotor} branch and lands in {@code hardwareMap.dcMotor}
     * alongside {@code FakeMotor}. That is right rather than a collision: an external encoder is plugged
     * into a motor port and is configured on a real robot as a motor, so a real {@code HardwareMap} holds
     * it in the same mapping. No other fake in this package matches two branches today; if one is added
     * that does, the order of this chain becomes a decision rather than an accident, and it should be
     * stated here at that point.
     *
     * <p><b>Sensors go in the way the SDK files a configured sensor.</b> A touch, colour or distance sensor
     * ({@link FakeTouchSensor}, {@link FakeRevColorSensorV3}, {@link FakeDistanceSensor}) takes the branch
     * that {@code SensorFiling} describes, which puts one device into every mapping it fits, as the real
     * configuration parser does - so a colour sensor V3 is in
     * {@code hardwareMap.colorSensor}, {@code hardwareMap.lightSensor} and
     * {@code hardwareMap.opticalDistanceSensor} at once.
     */
    private void putIntoTheMatchingTypedMapping(String name, HardwareDevice device) {
        if (device instanceof DcMotor) {
            dcMotor.put(name, (DcMotor) device);
        } else if (device instanceof CRServo) {
            crservo.put(name, (CRServo) device);
        } else if (device instanceof Servo) {
            servo.put(name, (Servo) device);
        } else if (device instanceof DigitalChannel) {
            digitalChannel.put(name, (DigitalChannel) device);
        } else if (device instanceof TouchSensor || device instanceof ColorSensor
                || device instanceof DistanceSensor) {
            SensorFiling.putIntoEveryMappingItFits(this, name, device);
        } else if (device instanceof AnalogInput) {
            analogInput.put(name, (AnalogInput) device);
        } else if (device instanceof VoltageSensor) {
            voltageSensor.put(name, (VoltageSensor) device);
        } else {
            put(name, device);
        }
    }

    /**
     * The SDK's own {@code internalPut} — every put, typed or not, ends here — followed by pinning the
     * enumeration order (class javadoc). The SDK's version clears its device list whenever a new device
     * arrives, to rebuild it from a {@code HashSet} on the next read; this puts back an ordered one instead.
     */
    @Override
    protected void internalPut(SerialNumber serialNumber, String deviceName, HardwareDevice device) {
        // Under the SDK's own lock, as its internalPut is, so a reader never sees the unordered list.
        synchronized (lock) {
            super.internalPut(serialNumber, deviceName, device);
            if (!containsThisExactObject(allDevicesInFirstPutOrder, device)) {
                allDevicesInFirstPutOrder.add(device);
            }
            allDevicesList = new ArrayList<>(allDevicesInFirstPutOrder);
        }
    }

    /**
     * The SDK's own {@code remove} — every removal ends here, including the one {@code DeviceMapping.put}
     * makes when a registration replaces a device — followed by pinning the enumeration order. A device
     * leaves the ordered list, and stops being ticked, only once it is registered under no name at all,
     * which is when the SDK's own list would lose it too.
     */
    @Override
    public boolean remove(SerialNumber serialNumber, String deviceName, HardwareDevice device) {
        synchronized (lock) {
            boolean removed = super.remove(serialNumber, deviceName, device);
            if (!isRegisteredUnderAnyName(device)) {
                for (int i = 0; i < allDevicesInFirstPutOrder.size(); i++) {
                    if (allDevicesInFirstPutOrder.get(i) == device) {
                        allDevicesInFirstPutOrder.remove(i);
                        break;
                    }
                }
            }
            allDevicesList = new ArrayList<>(allDevicesInFirstPutOrder);
            stopTickingDevicesNoLongerInTheMap();
            return removed;
        }
    }

    /** Whether {@code device} is still registered under at least one name in the inherited map. */
    private boolean isRegisteredUnderAnyName(Object device) {
        for (List<HardwareDevice> devicesUnderOneName : allDevicesMap.values()) {
            if (containsThisExactObject(devicesUnderOneName, device)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Whether {@code list} holds this very object — compared with {@code ==}, not {@code equals}, because
     * "the same device" here means the same fake instance, whatever a device class might define equality as.
     */
    private static boolean containsThisExactObject(List<?> list, Object object) {
        for (Object element : list) {
            if (element == object) {
                return true;
            }
        }
        return false;
    }

    /**
     * The SDK's own {@code tryGet}, minus the one line of it that cannot run off a robot.
     *
     * <p>{@code HardwareMap.tryGet} ends with a diagnostic that warns a team who asked for a BNO055 IMU on a
     * Control Hub that has a BHI260 — and it reaches that diagnostic through {@code Device.isRevControlHub()},
     * whose class initializer calls {@code android.os.Environment.getExternalStorageDirectory()}. On a JVM
     * that throws {@code "Method getExternalStorageDirectory in android.os.Environment not mocked"}, so the
     * inherited lookup cannot be used here at all: every {@code get} in every sim test would die inside the
     * SDK before it found anything. Overriding this one method is what lets everything else on
     * {@link HardwareMap} — {@code get}, {@code getAll}, {@code size}, iteration, the typed mappings — be the
     * SDK's own code reading the SDK's own storage.
     *
     * <p>The lookup itself is the SDK's, line for line: trim the name, walk the devices registered under it,
     * return the first one that is an instance of the requested type. The two things left out are the
     * Control-Hub IMU warning above, and {@code initializeDeviceIfNecessary}, which is private and only ever
     * does anything for an {@code I2cDeviceSynchDevice}: it calls that device's {@code initialize()} the first
     * time it is looked up. One fake here is one - {@code FakeGoBildaPinpoint} - and this map does not
     * initialize it. Nothing observable is lost: the Pinpoint driver's initialization is only its
     * {@code doInitialize()}, which sets a REV hub's I2C bus speed and which the fake makes a no-op; the rest
     * of the SDK's {@code initialize()} is bookkeeping (an "is initialized" flag, the SDK's I2C warning list,
     * a log line) that needs the Android runtime.
     *
     * @param classOrInterface the type the caller wants back
     * @param deviceName       the config name, trimmed here exactly as the SDK trims it
     * @return the first registered device under that name that is an instance of {@code classOrInterface},
     *         or null if there is none — the SDK's contract for this method
     */
    @Override
    public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        List<HardwareDevice> registeredUnderThatName = allDevicesMap.get(deviceName.trim());
        if (registeredUnderThatName == null) {
            return null;
        }
        for (HardwareDevice device : registeredUnderThatName) {
            if (classOrInterface.isInstance(device)) {
                return classOrInterface.cast(device);
            }
        }
        return null;
    }

    /**
     * Advances every registered fake device's plant by one harness tick, once each, in registration order.
     *
     * @param deltaTime how far to advance, in seconds; the harness passes its fixed tick
     */
    public void updateAll(double deltaTime) {
        for (FakeDevice device : devicesToTickInRegistrationOrder) {
            device.update(deltaTime);
        }
    }
}
