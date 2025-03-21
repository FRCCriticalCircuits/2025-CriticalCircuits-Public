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

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    private final StatusSignal<Angle> currentRotationSignal = leftMotorLeader.getPosition();
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

        config.Slot0.kP = ElevatorConstants.PID.kP;
        config.Slot0.kI = ElevatorConstants.PID.kI;
        config.Slot0.kD = ElevatorConstants.PID.kD;

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
        inputs.currentPosition = rotationsToDistance(currentRotationSignal.getValue());
        inputs.targetPosition = targetPosition;
    }

    /**
     * Set the target currentRotationSignal of the elevator
     *
     * @param target linear currentRotationSignal
     */
    @Override
    public void setTargetPosition(Distance target) {
        this.targetPosition = target;

        // Use normal positionvoltage control for now
        leftMotorLeader.setControl(positionVoltage.withPosition(
                distanceToRotations(target)
        ));

    }

    private Angle distanceToRotations(Distance d) {
        return Rotations.of(d.in(Inches) / (Math.PI * ElevatorConstants.SPROCKET_PD));
    }

    private Distance rotationsToDistance(Angle a) {
        return Inches.of(a.in(Rotations) * (Math.PI * ElevatorConstants.SPROCKET_PD));
    }
}
