package frc.robot.subsystems.elevatoreffector;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.HardwareMap;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor elevatorMotorSim = DCMotor.getKrakenX60(2);
    private final ElevatorSim elevatorSim;

    private Distance targetPosition = Meters.of(0);


    public ElevatorIOSim() {
        elevatorSim = new ElevatorSim(
                elevatorMotorSim,
                ElevatorConstants.SENSOR_TO_MECHANISM_RATIO,
                4.5,
                Inches.of(ElevatorConstants.SPROCKET_PD).in(Meters),
                0,
                ElevatorConstants.Physical.MAX_EXTENSION.in(Meters),
                true,
                0,
                0.01,
                0
        );

        // FIXME: one day, proper sim...
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // TODO: Tune sprocket PD
        inputs.currentPosition = Inches.of(0);
        inputs.targetPosition = Inches.of(0);
    }
}
