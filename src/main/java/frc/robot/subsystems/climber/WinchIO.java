package frc.robot.subsystems.climber;

public interface WinchIO {
    public enum WINCH_STATES{
        UP,
        DOWN,
        IDLE
    }
    
    default public void runWinch(double voltage) {};
}
