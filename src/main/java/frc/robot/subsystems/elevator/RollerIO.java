package frc.robot.subsystems.elevator;

public interface RollerIO {
    public static class RollerIOInputs {
        public boolean coralDetected = false; 
        public boolean algaeDetected = false;
    }

    public enum RollerMode{
        IN,
        OUT,
        C_OUT_LIGHT,
        HOLD,
        IDLE
    }

    public default void updateInputs(RollerIOInputs inputs, boolean lowVoltage) {}
    public default void overrideStates(RollerIOInputs inputs) {};

    public default void setMode(RollerMode mode) {};
}
