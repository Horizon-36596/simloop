package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.text.SimpleDateFormat;
import java.time.Clock;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneOffset;
import java.util.Calendar;
import java.util.Date;
import java.util.Locale;

/** Reads the real date and time the usual ways, for example to name a log file, and a fixed clock once. */
@Autonomous(name = "Wall Dates")
public class WallDatesAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        String logName = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date()) + ".csv";
        Instant started = Instant.now();
        LocalDateTime local = LocalDateTime.now();
        long zoned = Clock.systemUTC().millis();
        int hour = Calendar.getInstance().get(Calendar.HOUR_OF_DAY);
        Instant fixed = Instant.now(Clock.fixed(Instant.EPOCH, ZoneOffset.UTC)); // reads the clock it is given
        blackboard.put("names", logName + started + local + zoned + hour + fixed);
        waitForStart();
        while (opModeIsActive()) {
            idle();
        }
    }
}
