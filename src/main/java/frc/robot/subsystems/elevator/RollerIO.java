package frc.robot.subsystems.elevator;

public interface RollerIO {
    public static class RollerIOInputs {
        public boolean coralDetected;
        public boolean algaeDetected;
    }

    public enum RollerMode{
        IN,
        OUT
    }

    public default void updateInputs(RollerIOInputs inputs) {}

    public void setHatcher(Boolean enable);
    public void setIntake(Boolean enable);

    public void setMode(RollerMode mode);
}
