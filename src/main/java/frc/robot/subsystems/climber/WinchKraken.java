package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.Timer;

public class WinchKraken implements WinchIO {
    TalonFX winchKraken = new TalonFX(29);
    
    private TalonFXConfiguration winchConfig;

    public WinchKraken() {
        winchConfig = new TalonFXConfiguration();

        winchConfig.CurrentLimits.StatorCurrentLimit = 80; 
        winchConfig.CurrentLimits.SupplyCurrentLimit = 20; 
        winchConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 
        winchConfig.CurrentLimits.StatorCurrentLimitEnable = true; 
        
        winchConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        winchConfig.Feedback.SensorToMechanismRatio = 80;
        winchConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        winchConfig.Audio.AllowMusicDurDisable = true;

        winchKraken.getConfigurator().apply(winchConfig);
    }
    
    public void runWinch(double voltage) {
        winchKraken.setVoltage(voltage);
    }

    @Override
    public void blink(double frequency) {
        MusicTone t = new MusicTone(frequency);
        winchKraken.setControl(t);
    }
}
