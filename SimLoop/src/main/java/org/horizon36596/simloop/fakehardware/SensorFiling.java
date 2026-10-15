package org.horizon36596.simloop.fakehardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * How {@link FakeHardwareMap} files a sensor: exactly as the SDK 12.0 configuration parser files the sensors
 * it builds from their driver annotations ({@code HardwareFactory.addUserDeviceToMap}, read out of the
 * bytecode). The device goes into the overall map under its name, then into <b>every</b> typed mapping whose
 * type it is an instance of, unless that mapping already holds something under the name. So a
 * {@code RevTouchSensor} is in {@code hardwareMap.touchSensor}; a {@code RevColorSensorV3} is in
 * {@code colorSensor}, {@code lightSensor} and {@code opticalDistanceSensor}; and a REV 2m distance sensor,
 * which fits no typed mapping, is only in the overall map - which is where
 * {@code get(DistanceSensor.class, name)} looks.
 *
 * <p>A class of its own, rather than two more private methods on {@code FakeHardwareMap}, only so that
 * edits to that class elsewhere do not collide with this one.
 */
final class SensorFiling {

    private SensorFiling() {
    }

    /**
     * Files {@code device} under {@code name} the way the SDK files a configured sensor.
     *
     * @param map    the map to file it in
     * @param name   the config name; trimmed by the SDK, as on a robot
     * @param device the sensor
     */
    static void putIntoEveryMappingItFits(HardwareMap map, String name, HardwareDevice device) {
        map.put(name, device);
        for (HardwareMap.DeviceMapping<? extends HardwareDevice> mapping : map.allDeviceMappings) {
            if (mapping.getDeviceTypeClass().isInstance(device) && !mapping.contains(name)) {
                putLocalInto(mapping, name, device);
            }
        }
    }

    /** {@code mapping.putLocal}, with the cast the wildcard needs; the caller has checked the type. */
    private static <T extends HardwareDevice> void putLocalInto(HardwareMap.DeviceMapping<T> mapping,
            String name, HardwareDevice device) {
        mapping.putLocal(name, mapping.cast(device));
    }
}
