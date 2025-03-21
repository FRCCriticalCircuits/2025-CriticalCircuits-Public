package frc.robot.subsystems.elevatoreffector;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class WristConstants {
    public static final double CURRENT_LIMIT_SUPPLY = 20;
    public static final double CURRENT_LIMIT_STATOR = 100;
    public static final Angle ZERO_POSITION = Degrees.of(60);
    // FIXME: cancoder will cahnge this
    public static final double SENSOR_TO_MECHANISM_RATIO = 15; // Gear ratio to update
    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static class PID {
        // No gamepiece
        public static class Nothing {
            public static final double kP = 0.1;
            public static final double kI = 0.1;
            public static final double kD = 0.1;
        }
        // Coral
        public static class Coral {
            public static final double kP = 0.1;
            public static final double kI = 0.1;
            public static final double kD = 0.1;
        }
        public static class Algae {
            public static final double kP = 0.1;
            public static final double kI = 0.1;
            public static final double kD = 0.1;
        }

        }
}
