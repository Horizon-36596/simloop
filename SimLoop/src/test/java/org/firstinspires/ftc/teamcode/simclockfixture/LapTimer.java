package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.util.ElapsedTime;

/** A team's own timer class built on ElapsedTime: counts laps and remembers the last lap time. */
public class LapTimer extends ElapsedTime {

    private int laps;
    private double lastLapSeconds;

    /** Starts timing now. */
    public LapTimer() {
        super();
    }

    /** Ends a lap: records its length and starts the next. */
    public void lap() {
        lastLapSeconds = seconds();
        laps++;
        reset();
    }

    /** @return laps completed */
    public int laps() {
        return laps;
    }

    /** @return the last lap's length, in seconds */
    public double lastLapSeconds() {
        return lastLapSeconds;
    }
}
