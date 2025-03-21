package frc.robot.subsystems.elevatoreffector;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    static class ElevatorIOInputs {
        public Distance currentPosition;
        public Distance targetPosition;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setTargetPosition(Distance target) {}
}
