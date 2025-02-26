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
    private Boolean hatcherEnabled = false, intakeEnabled = false;
    
    private Debouncer coralDebouncer = new Debouncer(0.1);
    private Debouncer algaeDebouncer = new Debouncer(0.1);

    private RollerMode mode = RollerMode.IN;

    public RollerKraken(){
        m_hatcherMotor = new TalonFX(DeviceID.Angler.HATCHER_ID);
        m_intakeMotor = new TalonFX(DeviceID.Angler.INTAKE_ID);

        rollerConfiguration = new TalonFXConfiguration();

        // common settings
        rollerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.StatorCurrentLimit = PhysicalConstants.Elevator.CurrentLimits.ROLLER_CURRENT_LIMIT;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        rollerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // hatcher
        rollerConfiguration.Feedback.SensorToMechanismRatio = 3.86;
        rollerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_hatcherMotor.getConfigurator().apply(rollerConfiguration);

        // intake
        rollerConfiguration.Feedback.SensorToMechanismRatio = 2.0;
        rollerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_intakeMotor.getConfigurator().apply(rollerConfiguration);

        coralSensor = new TimeOfFlight(DeviceID.Sensor.CORAL_SENSOR);
        algaeSensor = new TimeOfFlight(DeviceID.Sensor.ALGAE_SENSOR);

        coralSensor.setRangingMode(RangingMode.Short, 25);
        algaeSensor.setRangingMode(RangingMode.Short, 25);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        if(algaeSensor.isRangeValid()) inputs.algaeDetected = algaeDebouncer.calculate(algaeSensor.getRange() < 50);
        if(coralSensor.isRangeValid()) inputs.coralDetected = coralDebouncer.calculate(coralSensor.getRange() < 50);

        SmartDashboard.putBoolean("algae", inputs.algaeDetected);
        SmartDashboard.putBoolean("coral", inputs.coralDetected);
        SmartDashboard.putBoolean("coralValid", coralSensor.isRangeValid());
        SmartDashboard.putBoolean("algaeValid", algaeSensor.isRangeValid());
        SmartDashboard.putNumber("coralRange", coralSensor.getRange());
        SmartDashboard.putNumber("algaeRange", algaeSensor.getRange());

        switch (mode) {
            case IN:
                if(hatcherEnabled){
                    if(inputs.coralDetected) m_hatcherMotor.setVoltage(0.1);
                    else m_hatcherMotor.setVoltage(0.2);
                }else{
                    m_hatcherMotor.stopMotor();
                }
        
                if(intakeEnabled){
                    if(inputs.algaeDetected) m_intakeMotor.setVoltage(0.2);
                    else m_intakeMotor.setVoltage(0.4);
                }else{
                    m_hatcherMotor.stopMotor();
                }

                break;
            default:
                if(hatcherEnabled) m_hatcherMotor.setVoltage(-0.2);
                else m_hatcherMotor.stopMotor();
                if(intakeEnabled) m_intakeMotor.setVoltage(-0.2);
                else m_intakeMotor.stopMotor();
        }
    }

    @Override
    public void setHatcher(Boolean enable) {
        this.hatcherEnabled = enable;
    }

    @Override
    public void setIntake(Boolean enable) {
        this.intakeEnabled = enable;
    }

    @Override
    public void setMode(RollerMode mode) {
        this.mode = mode;
    }
}
