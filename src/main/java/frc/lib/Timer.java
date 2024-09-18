package frc.lib;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;

public class Timer {
    double start = 0; 
    List<Pair<String, Double>> marks;
    public Timer() {
        start = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        marks = new ArrayList<Pair<String, Double>>();
    }

    public static double time_s() {
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    public void mark(String name) {
        marks.add(Pair.of(name, time_s()));
    }

    public boolean overrun(double timeout) {
        if (marks.size() == 0 || marks.get(marks.size() - 1).getSecond() - start < timeout) return false;
        return true;
    }

    public String print() {
        double running = start;
        String message = "";
        for (Pair<String, Double> mark : marks) {
            message += String.format("\n                [%s] %5f",mark.getFirst(), (mark.getSecond() - running));
            running = mark.getSecond();
        }

        return message;
    }
}
