import java.io.PrintWriter;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.Locale;

import org.horizon36596.simloop.fit.RlogTimeSeries;

/** Exports the documented slide example's RLOG to the CSV used by its plot. */
class ExportSlide {
    public static void main(String[] args) throws Exception {
        if (args.length != 2) {
            throw new IllegalArgumentException("Usage: ExportSlide.java <input.rlog> <output.csv>");
        }
        String key = "RealOutputs/Slide/positionInches";
        RlogTimeSeries series = RlogTimeSeries.read(Paths.get(args[0]), Collections.singletonList(key));
        double[] time = series.timeSeconds();
        double[] position = series.column(key);
        try (PrintWriter out = new PrintWriter(args[1], "UTF-8")) {
            out.println("time_s,position_in,target_in");
            for (int i = 0; i < time.length; i++) {
                out.printf(Locale.ROOT, "%.6f,%.9f,12.0%n", time[i], position[i]);
            }
        }
        System.out.println("Exported " + series.size() + " frames; final position "
                + position[position.length - 1] + " in.");
    }
}
