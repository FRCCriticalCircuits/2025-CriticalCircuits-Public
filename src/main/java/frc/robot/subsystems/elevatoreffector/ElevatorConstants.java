package frc.robot.subsystems.elevatoreffector;

import com.ctre.phoenix6.signals.InvertedValue;

public class ElevatorConstants {
    public static final double CURRENT_LIMIT_SUPPLY = 40;
    public static final double CURRENT_LIMIT_STATOR = 120;
    public static final double SENSOR_TO_MECHANISM_RATIO = 15; // Gear ratio on MP
    public static final double SPROCKET_PD = 1.71; // For conversion to linear
    public static final InvertedValue INVERT_LEFT = InvertedValue.CounterClockwise_Positive;

    public class Heights {
        public static final double MAX_EXTENSION = 20; // Inches
        public static final double IDLE = 2; // Inches
    }
}
