package org.firstinspires.ftc.teamcode.simclockfixture;

import com.qualcomm.robotcore.util.ElapsedTime;

/** A team subclass that overrides nsNow and calls super.nsNow() (invokespecial ElapsedTime.nsNow). */
public class SuperNsNowTimer extends ElapsedTime {
    public SuperNsNowTimer() {
        super();
    }
    @Override
    protected long nsNow() {
        return super.nsNow() + 7;
    }
    public long rawNow() {
        return nsNow();
    }
}
