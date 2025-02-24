package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.subsystems.elevator.RollerSubsystem;

public class intakeCoral extends Command{
    private RollerSubsystem rollerSubsystem;
    private double timeEnds;
    
    public intakeCoral(double timeLimitSeconds){
        rollerSubsystem = RollerSubsystem.getInstance();
        this.timeEnds = Timer.getFPGATimestamp() + timeLimitSeconds;

        addRequirements(rollerSubsystem);
    }

    public void initialize(){
        rollerSubsystem.setIntakeCoral();
    }

    @Override
    public void end(boolean interrupted) {
        rollerSubsystem.idle();
        
        if(Robot.isSimulation()){
            RollerIOInputs newInputs = new RollerIOInputs();
            newInputs.algaeDetected = false;
            newInputs.coralDetected = true;
            rollerSubsystem.overrideSimStates(newInputs);
        }
    }

    @Override
    public boolean isFinished() {
        return rollerSubsystem.coralDetected() || (Timer.getFPGATimestamp() > timeEnds);
    }
}
