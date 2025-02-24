package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.RollerSubsystem;

public class shoot extends Command{
    private RollerSubsystem rollerSubsystem;
    private double timeEnds;
    
    public shoot(double timeLimitSeconds){
        rollerSubsystem = RollerSubsystem.getInstance();
        this.timeEnds = Timer.getFPGATimestamp() + timeLimitSeconds;

        addRequirements(rollerSubsystem);
    }

    public void initialize(){
        rollerSubsystem.outTake();
    }

    @Override
    public void end(boolean interrupted) {
        rollerSubsystem.idle();
    }

    @Override
    public boolean isFinished() {
        return (!rollerSubsystem.coralDetected() && !rollerSubsystem.algaeDetected()) || (Timer.getFPGATimestamp() > timeEnds);
    }
}
