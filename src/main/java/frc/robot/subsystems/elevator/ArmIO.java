package frc.robot.subsystems.elevator;

public interface ArmIO {
    public static class ArmIOInputs {
        public double rotation;
        public double targetRotation;
        
        public boolean coralDetected;
        public boolean algaeDetected;
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    public void setRotation(double rotation);
}
