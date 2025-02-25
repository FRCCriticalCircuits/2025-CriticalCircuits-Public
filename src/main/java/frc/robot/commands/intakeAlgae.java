package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerSubsystem;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;

public class intakeAlgae extends Command{
    private RollerSubsystem rollerSubsystem;
    private double timeEnds;
    
    public intakeAlgae(double timeLimitSeconds){
        rollerSubsystem = RollerSubsystem.getInstance();
        this.timeEnds = Timer.getFPGATimestamp() + timeLimitSeconds;

        addRequirements(rollerSubsystem);
    }

    public void initialize(){
        rollerSubsystem.setIntakeAlgae();
    }

    @Override
    public void end(boolean interrupted) {
        rollerSubsystem.idle();
        if(Robot.isSimulation()){
            RollerIOInputs newInputs = new RollerIOInputs();
            newInputs.algaeDetected = true;
            newInputs.coralDetected = false;
            rollerSubsystem.overrideSimStates(newInputs);
        }
    }

    @Override
    public boolean isFinished() {
        return rollerSubsystem.algaeDetected() || (Timer.getFPGATimestamp() > timeEnds);
    }
}
