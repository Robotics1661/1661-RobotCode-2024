package frc.robot.util;

import com.ctre.phoenix6.Utils;

public class SimulationDebugger {
    public static void autoDbg(String msg) {
        if (Utils.isSimulation() || true) {
            System.out.println("\n\n\t\t"+msg+"\n\n");
        }
    }
}
