package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
    public static class ArmIOInputs {
        public Rotation2d ioRotation;
        public Rotation2d targetRotation;
    }

    public default void updateInputs(ArmIOInputs inputs, boolean coralDetected, boolean algaeDetected) {}
    public void setRotation(Rotation2d rotation);
}
