package frc.robot.subsystems.swerve.swerveHAL;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.wpilibj.Timer;

public class SimUtil {
     public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] = Timer.getFPGATimestamp()
                    - 0.02
                    + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }
}
