package frc.robot.subsystems.elevatoreffector;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public class ElevatorConstants {
    public static final double CURRENT_LIMIT_SUPPLY = 40;
    public static final double CURRENT_LIMIT_STATOR = 120;
    public static final double SENSOR_TO_MECHANISM_RATIO = 15; // Gear ratio on MP
    public static final double SPROCKET_PD = 1.71; // For conversion to linear
    public static final InvertedValue INVERT_LEFT = InvertedValue.CounterClockwise_Positive;

    public static class Physical {
        public static final Distance MAX_EXTENSION = Inches.of(20); // Inches
        public static final Angle ALGAE_PICK = Degrees.of(-30);
        public static final Pair<Distance, Angle> ZERO = new Pair<>(Inches.zero(), Degrees.zero());
        public static final Pair<Distance, Angle> IDLE = new Pair<>(Inches.of(2), Degrees.of(0));
        public static final Pair<Distance, Angle> L1_CORAL = new Pair<>(Inches.of(0), Degrees.of(0));
        public static final Pair<Distance, Angle> L2_CORAL = new Pair<>(Inches.of(0), Degrees.of(0));
        public static final Pair<Distance, Angle> L3_CORAL = new Pair<>(Inches.of(0), Degrees.of(0));
        public static final Pair<Distance, Angle> L4_CORAL = new Pair<>(Inches.of(0), Degrees.of(0));
        public static final Pair<Distance, Angle> INTAKE_CORAL = new Pair<>(Inches.of(0), Degrees.of(0));

        public static final Pair<Distance, Angle> L1_ALGAE = new Pair<>(Inches.of(0), Degrees.of(0));
        public static final Pair<Distance, Angle> L2_ALGAE = new Pair<>(Inches.of(0), Degrees.of(0));
        public static final Pair<Distance, Angle> INTAKE_ALGAE = new Pair<>(Inches.of(0), Degrees.of(0));
        public static final Pair<Distance, Angle> BARGE_ALGAE = new Pair<>(Inches.of(0), Degrees.of(0));

        public static final Pair<Distance, Angle> T1 = new Pair<>(Inches.of(2), Degrees.of(0));
        public static final Pair<Distance, Angle> T2 = new Pair<>(Inches.of(0), Degrees.of(0));
        public static final Pair<Distance, Angle> T3 = new Pair<>(Inches.of(0), Degrees.of(0));
    }
}
