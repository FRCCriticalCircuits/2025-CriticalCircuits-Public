package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.climber.WinchIO.WINCH_STATES;

public class WinchSubsystem extends SubsystemBase {
    private static WinchSubsystem instance;
    private WinchIO winchIO;
    
    public WinchSubsystem() {
        if(Robot.isSimulation()){
            winchIO = new WinchSim();
        }else{
            winchIO = new WinchKraken();
        }
    }
    
    public void setState(WINCH_STATES state) {
        switch (state) {
            case DOWN:
                winchIO.runWinch(4);
                break;
            case UP:
                winchIO.runWinch(-4);
                break;
            case IDLE:
                winchIO.runWinch(0);
                break;
        }
    }

    public static WinchSubsystem getInstance() {
        if (instance == null) instance = new WinchSubsystem();
        return instance;
    }
}
