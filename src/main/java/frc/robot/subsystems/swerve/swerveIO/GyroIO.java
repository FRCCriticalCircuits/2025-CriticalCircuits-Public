package frc.robot.subsystems.swerve.swerveIO;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }
    
    public default void updateInputs(GyroIOInputs inputs) {}
    
    /**
     * reset angle of the gyro
     * @param yaw yaw angle in rotations
     */
    public void setYaw(double rotation);
}
