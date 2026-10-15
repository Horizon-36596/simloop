package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The loop of the SDK's ConceptRampMotorSpeed and ConceptScanServo samples: set a power, {@code sleep(50)},
 * {@code idle()}. A constant sleep, so it runs the same code both as an instance and as a team class.
 */
@Autonomous(name = "Sleep Then Idle")
public class SleepThenIdleAuto extends LinearOpMode {

    private static final int CYCLE_MS = 50;

    @Override
    public void runOpMode() {
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        double power = 0;
        waitForStart();
        while (opModeIsActive()) {
            power += 0.01;
            motor.setPower(power);
            telemetry.addData("power", "%.2f", power);
            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }
    }
}
