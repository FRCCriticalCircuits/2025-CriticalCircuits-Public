package frc.robot.subsystems.elevatoreffector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface WristIO {
    public enum WristIOState {
        NOTHING, CORAL, ALGAE
    }

    @AutoLog
    public static class WristIOInputs {
        public Angle currentRotation;
        public Angle targetRotation;
    }

    public default void updateInputs(WristIOInputs inputs) {

    }

    public default void setTargetAngle(Angle angle) {

    }

    /**
     * Change the PID gain based on the mode
     * @param state what game piece is in end effector
     */
    public default void setMode(WristIOState state) {

    }
}
