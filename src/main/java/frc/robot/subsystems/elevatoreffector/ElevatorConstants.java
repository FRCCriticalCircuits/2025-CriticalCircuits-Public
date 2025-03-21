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

    public static final class PID {
        public static final double kP = 20;
        public static final double kI = 0;
        public static final double kD = 5;
    }

    public static class Physical {
        // The real angle read from TalonFX at 0 in degrees
        // FIXME: this shouldnt be needed once we install a cancoder
        public static final Angle ZERO_OFFSET = Rotations.of(0.4);
        public static final Distance MAX_EXTENSION = Inches.of(20);
        public static final Angle ALGAE_PICK = Degrees.of(-30);
        public static final Pair<Distance, Angle> ZERO = new Pair<>(Inches.zero(), ZERO_OFFSET);
        public static final Pair<Distance, Angle> IDLE = new Pair<>(Inches.of(1), Degrees.of(0));
        public static final Pair<Distance, Angle> L1_CORAL = new Pair<>(Inches.of(1), Degrees.of(0));
        public static final Pair<Distance, Angle> L2_CORAL = new Pair<>(Inches.of(3), Degrees.of(0));
        // Actually also a transition state, all nodes
        // go here before going any higher (BARGE, INTAKE)
        public static final Pair<Distance, Angle> L3_CORAL = new Pair<>(Inches.of(10), Degrees.of(0));
        public static final Pair<Distance, Angle> L4_CORAL = new Pair<>(Inches.of(10), Degrees.of(0));
        public static final Pair<Distance, Angle> STATION_INTAKE_CORAL = new Pair<>(Inches.of(10), Degrees.of(0));

        public static final Pair<Distance, Angle> L1_ALGAE = new Pair<>(Inches.of(3), Degrees.of(0));
        public static final Pair<Distance, Angle> L1_ALGAE_IN = new Pair<>(Inches.of(3), Degrees.of(-15));
        public static final Pair<Distance, Angle> L2_ALGAE = new Pair<>(Inches.of(10), Degrees.of(0));
        public static final Pair<Distance, Angle> L2_ALGAE_IN = new Pair<>(Inches.of(10), Degrees.of(-15));
        public static final Pair<Distance, Angle> GROUND_INTAKE_ALGAE = new Pair<>(Inches.of(0), Degrees.of(0));
        public static final Pair<Distance, Angle> BARGE_ALGAE = new Pair<>(Inches.of(0), Degrees.of(0));

        // First transition near idle state before elevator is able to move to avoid collision
        public static final Pair<Distance, Angle> T1 = new Pair<>(Inches.of(2), Degrees.of(0));
        public static final Pair<Distance, Angle> T2 = new Pair<>(Inches.of(0), Degrees.of(0));
        public static final Pair<Distance, Angle> T3 = new Pair<>(Inches.of(0), Degrees.of(0));
    }
}
