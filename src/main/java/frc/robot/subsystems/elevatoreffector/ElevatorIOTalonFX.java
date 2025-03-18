package frc.robot.subsystems.elevatoreffector;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.HardwareMap;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leftMotorLeader = new TalonFX(HardwareMap.CAN.ELEVATOR_LEFT_ID);
    private final TalonFX rightMotorFollower = new TalonFX(HardwareMap.CAN.ELEVATOR_RIGHT_ID);

    private PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    private final StatusSignal<Angle> position = leftMotorLeader.getPosition();
    private Distance targetPosition = Meters.of(0);


    public ElevatorIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.CURRENT_LIMIT_STATOR;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.CURRENT_LIMIT_SUPPLY;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.SENSOR_TO_MECHANISM_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = ElevatorConstants.INVERT_LEFT;

        StatusCode status = leftMotorLeader.getConfigurator().apply(config, 0.25);
        // Show error if failed to init
        if (status != StatusCode.OK) {
            new Alert("Failed to configure TalonFX (Left): " + this.getClass().getSimpleName(),
                    Alert.AlertType.kError).set(true);
        }

        status = rightMotorFollower.getConfigurator().apply(config, 0.25);
        // Show error if failed to init
        if (status != StatusCode.OK) {
            new Alert("Failed to configure TalonFX (Right): " + this.getClass().getSimpleName(),
                    Alert.AlertType.kError).set(true);
        }

        // Set follower to be the right motor and reverse
        rightMotorFollower.setControl(new Follower(leftMotorLeader.getDeviceID(), true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // TODO: Tune sprocket PD
        inputs.currentPosition = Inches.of(position.getValueAsDouble() *
                Math.PI * ElevatorConstants.SPROCKET_PD);
        inputs.targetPosition = targetPosition;
    }

    /**
     * Set the target position of the elevator
     * @param target linear position
     */
    @Override
    public void setTargetPosition(Distance target) {
        this.targetPosition = target;
        leftMotorLeader.setControl(positionVoltage.withPosition(
                distanceToRotations(target)
        ));
    }

    private double distanceToRotations(Distance d) {
        return d.in(Inches) / (Math.PI * ElevatorConstants.SPROCKET_PD);
    }

    private double rotationsToDistance(Angle a) {
        return a.in(Rotations) * (Math.PI * ElevatorConstants.SPROCKET_PD);
    }
}
