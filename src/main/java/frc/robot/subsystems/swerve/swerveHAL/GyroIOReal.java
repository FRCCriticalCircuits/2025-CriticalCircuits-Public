package frc.robot.subsystems.swerve.swerveHAL;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DeviceID;

public class GyroIOReal implements GyroIO{
    private final Canandgyro gyro;

    public GyroIOReal(){
        gyro = new Canandgyro(DeviceID.DriveBase.GYRO_CAN_ID);

        Commands.print("[Swerve] Gyro Calibrating").schedule();
        gyro.waitForCalibrationToFinish(5);
        Commands.print("[Swerve] Gyro Calibration Finished").schedule();
    }

    @Override
    public Rotation2d getGyroRotation2D() {
        return gyro.getRotation2d();
    }

    /**
     * reset angle of gyro
     * @param yaw new angle in rotations
     */
    @Override
    public void setYaw(double yaw) {
        gyro.setYaw(yaw);
    }

    /**
     * @return speed in Radians per Sec
     */
    @Override
    public double getYawVelocity() {
        return gyro.getAngularVelocityYaw() * Math.PI * 2;   
    }
}
