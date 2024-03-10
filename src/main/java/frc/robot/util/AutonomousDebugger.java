package frc.robot.util;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutonomousDebugger {
    private static final DecimalFormat DECIMAL_FORMAT = new DecimalFormat("00.000");
    private static double startTime;
    private static final List<LogEntry> log = new ArrayList<>();

    public static Command markAutoStart() {
        return new InstantCommand(() -> {
            log.clear();
            startTime = Timer.getFPGATimestamp();
        });
    }

    public static Command printAutoEnd() {
        return new InstantCommand(() -> {
            double duration = Timer.getFPGATimestamp() - startTime;
            autoDbg("Auto complete!");
            autoDbg("Auto duration: "+duration+" seconds");

            
            System.out.println("\n\n\t\tAuto event log:");
            for (LogEntry entry : log) {
                System.out.println("\t\t[%s]\t%s".formatted(DECIMAL_FORMAT.format(entry.time), entry.msg));
            }
            System.out.println("\n\n");
        });
    }

    public static void autoDbg(String msg) {
        System.out.println("\n\n\t\t"+msg+"\n\n");
        log.add(new LogEntry(Timer.getFPGATimestamp() - startTime, msg));
    }

    private static record LogEntry(double time, String msg) {}
}
