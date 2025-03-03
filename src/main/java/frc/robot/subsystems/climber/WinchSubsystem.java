package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import pabeles.concurrency.IntRangeConsumer;

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
    
    public void winch(int state) {
        if (state == 1) {
            winchIO.runWinch(12);
        } else if (state == -1) {
            winchIO.runWinch(-12);
        } else {
            winchIO.runWinch(0);
        }
    }

    public static WinchSubsystem getInstance() {
        if (instance == null) instance = new WinchSubsystem();

        return instance;
    }
}
