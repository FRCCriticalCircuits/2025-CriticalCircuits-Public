package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    public static class ElevatorIOInputs {
        public double position;
        public double targetPosition;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public void setPosition(double rotation);
}