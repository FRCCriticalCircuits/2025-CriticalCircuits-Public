package frc.robot.subsystems.swerve.swerveHAL;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO{
    private GyroSimulation gyroSimulation;

    public GyroIOSim(){
        this.gyroSimulation = COTS.ofPigeon2().get();
    }

    @Override
    public Rotation2d getGyroRotation2D() {
        return this.gyroSimulation.getGyroReading();
    }

    /**
     * reset angle of gyro
     * @param yaw new angle in rotations
     */
    @Override
    public void setYaw(double yaw) {
        this.gyroSimulation.setRotation(Rotation2d.fromRotations(yaw));
    }
}
