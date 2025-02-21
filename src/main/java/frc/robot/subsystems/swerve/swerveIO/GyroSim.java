package frc.robot.subsystems.swerve.swerveIO;

import static edu.wpi.first.units.Units.*;

import java.util.Queue;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.MotorUtil;
public class GyroSim implements GyroIO{
    private final GyroSimulation gyroSimulation;

    public GyroSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
        gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

        inputs.odometryYawTimestamps = MotorUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }

    @Override
    public void setYaw(double rotation) {
        this.gyroSimulation.setRotation(Rotation2d.fromRotations(rotation));
    }
}
