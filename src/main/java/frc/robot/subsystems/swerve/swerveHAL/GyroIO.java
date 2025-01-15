package frc.robot.subsystems.swerve.swerveHAL;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    public Rotation2d getGyroRotation2D();
    public void setYaw(double yaw);
}
