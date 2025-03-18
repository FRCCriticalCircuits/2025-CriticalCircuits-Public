package frc.robot.subsystems.elevatoreffector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public Distance currentPosition;
        public Distance targetPosition;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setTargetPosition(Distance target) {}
}
