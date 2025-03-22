package frc.robot.subsystems.elevatoreffector;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.Inches;

public interface ElevatorIO {
    @AutoLog
    static class ElevatorIOInputs {
        public Distance currentPosition = Inches.of(0);
        public Distance targetPosition = Inches.of(0);
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setTargetPosition(Distance target) {}
}
