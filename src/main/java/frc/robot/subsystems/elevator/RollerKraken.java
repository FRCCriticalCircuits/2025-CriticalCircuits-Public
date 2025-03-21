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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Constants.DeviceID;
import frc.robot.Constants.Physical;
import frc.robot.commands.swerve.TempDriveOffsetCommand;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.utils.structures.DataStrcutures.Mode;

public class RollerKraken implements RollerIO {
    private TalonFX hatcherMotor;
    private TalonFX algaeMotor;

    private TalonFXConfiguration rollerConfiguration;

    private TimeOfFlight algaeSensor;

    private Debouncer coralDebouncer = new Debouncer(0.1);
    private Debouncer algaeDebouncer = new Debouncer(0.1);

    private RollerMode mode = RollerMode.HOLD;

    private VelocityVoltage hatcherControl = new VelocityVoltage(0).withSlot(0);
    private Robot robot;

    private boolean sameCoral = false;

    public RollerKraken(Robot r){
        this.robot = r;
        hatcherMotor = new TalonFX(DeviceID.Angler.HATCHER_ID);
        algaeMotor = new TalonFX(DeviceID.Angler.INTAKE_ID);

        rollerConfiguration = new TalonFXConfiguration();

        // common settings
        rollerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.StatorCurrentLimit = Physical.Elevator.CurrentLimits.ROLLER_CURRENT_LIMIT;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimit = 5;

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

        hatcherMotor.getConfigurator().apply(rollerConfiguration);

        // intake
        rollerConfiguration.Feedback.SensorToMechanismRatio = 2.0;
        rollerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        algaeMotor.getConfigurator().apply(rollerConfiguration);

        algaeSensor = new TimeOfFlight(DeviceID.Sensor.ALGAE_SENSOR);

        algaeSensor.setRangingMode(RangingMode.Short, 25);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs, boolean lowVoltage) {
        // inputs.algaeDetected = algaeDebouncer.calculate(
        //     algaeSensor.isRangeValid() &&
        //     algaeSensor.getRange() < 90
        // );
        inputs.algaeDetected = algaeDebouncer.calculate(
            algaeMotor.getVelocity().getValueAsDouble() < 1.5
        ) && DriveStationIO.isEnabled();

        // Debounce 100ms due to velocity error
        inputs.coralDetected = coralDebouncer.calculate(
            hatcherMotor.getVelocity().getValueAsDouble() < 1.5
        ) && DriveStationIO.isEnabled();

        SmartDashboard.putBoolean("algae", inputs.algaeDetected);
        SmartDashboard.putBoolean("coral", inputs.coralDetected);

        if (inputs.coralDetected && 
            (AutoAimManager.getInstance().getSetting().getMode() == Mode.CORAL_INTAKE 
                || AutoAimManager.getInstance().getSetting().getMode() == Mode.CORAL_PLACE
            ) && hatcherMotor.getDeviceEnable().getValue().value == 1.0
            && robot.isTeleopEnabled()
            
        ) {
            if (!sameCoral) {
                sameCoral = true;
                // Move the robot back and change the elevator state automatically
                new TempDriveOffsetCommand(0, -1).withTimeout(0.25).andThen(
                  new InstantCommand(() -> {
                    // Change the mode back to coral placement
                    AutoAimManager.getInstance().updateMode(Mode.CORAL_PLACE);  
                  })
                );
            }
            LEDSubsystem.getInstance().setBlink(true);
        } else {
            sameCoral = false;
            LEDSubsystem.getInstance().setBlink(false);
        }
    }

    @Override
    public void setMode(RollerMode mode) {
        this.mode = mode;

        switch (mode) {
            case CORAL_IN -> hatcherMotor.setControl(hatcherControl.withVelocity(10));
            case CORAL_OUT -> hatcherMotor.setControl(hatcherControl.withVelocity(-20));
            case ALGAE_IN -> {
                hatcherMotor.setControl(hatcherControl.withVelocity(10));
                algaeMotor.setControl(hatcherControl.withVelocity(10));
            }
            case ALGAE_OUT -> {
                algaeMotor.setControl(hatcherControl.withVelocity(-10));
            }
            case CORAL_OUT_LIGHT -> {
                hatcherMotor.setControl(hatcherControl.withVelocity(-4.5));
                algaeMotor.stopMotor();
            }
            case HOLD -> {
                hatcherMotor.setControl(hatcherControl.withVelocity(3));
                algaeMotor.stopMotor();
            }
            case HOLD_ALGAE ->  {
                hatcherMotor.setControl(hatcherControl.withVelocity(3));
                algaeMotor.setControl(hatcherControl.withVelocity(3));
            }
        }
    }
}
