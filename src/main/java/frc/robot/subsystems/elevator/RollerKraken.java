package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DeviceID;
import frc.robot.Constants.Physical;

public class RollerKraken implements RollerIO {
    private TalonFX m_hatcherMotor;
    private TalonFX m_intakeMotor;

    private TalonFXConfiguration rollerConfiguration;

    private TimeOfFlight algaeSensor;

    private Debouncer coralDebouncer = new Debouncer(0.1);
    private Debouncer algaeDebouncer = new Debouncer(0.1);

    private RollerMode mode = RollerMode.IN;

    private VelocityVoltage hatcherControl = new VelocityVoltage(0).withSlot(0);

    public RollerKraken(){
        m_hatcherMotor = new TalonFX(DeviceID.Angler.HATCHER_ID);
        m_intakeMotor = new TalonFX(DeviceID.Angler.INTAKE_ID);

        rollerConfiguration = new TalonFXConfiguration();

        // common settings
        rollerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.StatorCurrentLimit = Physical.Elevator.CurrentLimits.ROLLER_CURRENT_LIMIT;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimit = 20;

        rollerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // hatcher
        rollerConfiguration.Feedback.SensorToMechanismRatio = 3.86;
        rollerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfiguration.Slot0 = new Slot0Configs()
            .withKV(0.35)
            .withKS(0.3)
            .withKP(1)
            .withKD(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

        m_hatcherMotor.getConfigurator().apply(rollerConfiguration);

        // intake
        rollerConfiguration.Feedback.SensorToMechanismRatio = 2.0;
        rollerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_intakeMotor.getConfigurator().apply(rollerConfiguration);

        algaeSensor = new TimeOfFlight(DeviceID.Sensor.ALGAE_SENSOR);

        algaeSensor.setRangingMode(RangingMode.Short, 25);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.algaeDetected = algaeDebouncer.calculate(
            algaeSensor.isRangeValid() &&
            algaeSensor.getRange() < 90
        );

        inputs.coralDetected = coralDebouncer.calculate(
            m_hatcherMotor.getVelocity().getValueAsDouble() < 1.0 &&
            m_hatcherMotor.getSupplyCurrent().getValueAsDouble() > 2.0 &&
            // limitSwitch.get() &&
            (mode == RollerMode.IN)
        );

        SmartDashboard.putBoolean("algae", inputs.algaeDetected);
        SmartDashboard.putBoolean("coral", inputs.coralDetected);

        switch (mode) {
            case IN:
                m_hatcherMotor.setControl(hatcherControl.withVelocity(10));
                m_intakeMotor.setVoltage(4.0);
                break;
            case OUT:
                m_hatcherMotor.setVoltage(-12.0);
                m_intakeMotor.setVoltage(-12.0);
                break;
            case HOLD:
                m_hatcherMotor.setVoltage(0.70);
                m_intakeMotor.setVoltage(0.70);
                break;
            case IDLE:
                m_hatcherMotor.stopMotor();
                m_intakeMotor.stopMotor();
                break;
        }
    }

    @Override
    public void setMode(RollerMode mode) {
        this.mode = mode;
    }
}
