package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DeviceID;
import frc.robot.Constants.PhysicalConstants;

public class RollerKraken implements RollerIO {
    private TalonFX m_hatcherMotor;
    private TalonFX m_intakeMotor;

    private TalonFXConfiguration rollerConfiguration;

    private TimeOfFlight coralSensor, algaeSensor;
    
    private Debouncer coralDebouncer = new Debouncer(0.1);
    private Debouncer algaeDebouncer = new Debouncer(0.1);

    private RollerMode mode = RollerMode.HOLD;

    public RollerKraken(){
        m_hatcherMotor = new TalonFX(DeviceID.Angler.HATCHER_ID);
        m_intakeMotor = new TalonFX(DeviceID.Angler.INTAKE_ID);

        rollerConfiguration = new TalonFXConfiguration();

        // common settings
        rollerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.StatorCurrentLimit = PhysicalConstants.Elevator.CurrentLimits.ROLLER_CURRENT_LIMIT;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        rollerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // hatcher
        rollerConfiguration.Feedback.SensorToMechanismRatio = 3.86;
        rollerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_hatcherMotor.getConfigurator().apply(rollerConfiguration);

        // intake
        rollerConfiguration.Feedback.SensorToMechanismRatio = 2.0;
        rollerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_intakeMotor.getConfigurator().apply(rollerConfiguration);

        coralSensor = new TimeOfFlight(DeviceID.Sensor.CORAL_SENSOR);
        algaeSensor = new TimeOfFlight(DeviceID.Sensor.ALGAE_SENSOR);

        coralSensor.setRangingMode(RangingMode.Short, 25);
        algaeSensor.setRangingMode(RangingMode.Short, 25);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.algaeDetected = algaeDebouncer.calculate(
            algaeSensor.isRangeValid() &&
            algaeSensor.getRange() < 80
        );
        
        inputs.coralDetected = coralDebouncer.calculate(
            coralSensor.isRangeValid() &&
            coralSensor.getRange() < 50
        );

        SmartDashboard.putBoolean("algae", inputs.algaeDetected);
        SmartDashboard.putBoolean("coral", inputs.coralDetected);

        switch (mode) {
            case IN:
                m_hatcherMotor.setVoltage(4.0);
                m_intakeMotor.setVoltage(4.0);  
                break;
            case OUT:
                m_hatcherMotor.setVoltage(-5.0);
                m_intakeMotor.setVoltage(-5.0);
            case HOLD:
                m_hatcherMotor.setVoltage(0);
                m_intakeMotor.setVoltage(0);
            case IDLE:
                m_hatcherMotor.stopMotor();
                m_intakeMotor.stopMotor();
        }
    }

    @Override
    public void setMode(RollerMode mode) {
        this.mode = mode;
    }
}
