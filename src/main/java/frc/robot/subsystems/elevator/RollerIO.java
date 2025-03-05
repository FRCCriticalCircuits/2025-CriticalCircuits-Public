package frc.robot.subsystems.elevator;

public interface RollerIO {
    public static class RollerIOInputs {
        public boolean coralDetected = false; 
        public boolean algaeDetected = false;
    }

    public enum RollerMode{
        IN,
        OUT,
        HOLD,
        IDLE
    }

    public default void updateInputs(RollerIOInputs inputs) {}
    public default void overrideStates(RollerIOInputs inputs) {};

    public default void setMode(RollerMode mode) {};
}
