package frc.robot.subsystems.elevator;

public interface RollerIO {
    public static class RollerIOInputs {
        public boolean coralDetected = false; 
        public boolean algaeDetected = false;
    }

    public enum RollerMode{
        IN,
        CORAL_IN,
        OUT,
        CORAL_OUT,
        ALGAE_IN,
        ALGAE_OUT,
        CORAL_OUT_LIGHT,
        HOLD,
        HOLD_ALGAE
    }

    public default void updateInputs(RollerIOInputs inputs, boolean lowVoltage) {}
    public default void overrideStates(RollerIOInputs inputs) {};

    public default void setMode(RollerMode mode) {};
}
