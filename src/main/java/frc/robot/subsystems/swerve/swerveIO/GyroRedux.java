package frc.robot.subsystems.swerve.swerveIO;

import java.util.Queue;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DeviceID;

public class GyroRedux implements GyroIO{
    private Canandgyro gyro;

    private Queue<Double> yawPositionQueue;
    private Queue<Double> yawTimestampQueue;

    public GyroRedux(){
        gyro = new Canandgyro(DeviceID.DriveBase.GYRO_CAN_ID);

        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(gyro::getYaw);

        Commands.print("[Swerve] Gyro Calibrating").schedule();
        gyro.waitForCalibrationToFinish(5);
        Commands.print("[Swerve] Gyro Calibration Finished").schedule();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = gyro.getRotation2d();
        inputs.yawVelocityRadPerSec = gyro.getAngularVelocityYaw() * Math.PI * 2;

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRotations(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
    
    @Override
    public void setYaw(double rotation) {
        gyro.setYaw(rotation);
    }
}
