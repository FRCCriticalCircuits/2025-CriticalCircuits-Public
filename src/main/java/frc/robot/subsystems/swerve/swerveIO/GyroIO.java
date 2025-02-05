package frc.robot.subsystems.swerve.swerveIO;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    /**
     * Get Rotation of Gyro
     * @return {@link Rotation2d} object
     */
    public Rotation2d getGyroRotation2D();
    /**
     * reset angle of the gyro
     * @param yaw yaw angle in rotations
     */
    public void setYaw(double yaw);
    /**
     * get velocity of gyro yaw
     * @return velocity in Radians per Sec
     */
    public double getYawVelocity();
}
