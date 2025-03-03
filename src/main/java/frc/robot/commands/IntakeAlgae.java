package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerSubsystem;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;

public class IntakeAlgae extends Command{
    private RollerSubsystem rollerSubsystem;
    
    public IntakeAlgae(){
        rollerSubsystem = RollerSubsystem.getInstance();

        addRequirements(rollerSubsystem);
    }

    public void initialize(){
        rollerSubsystem.intake();
    }

    @Override
    public void end(boolean interrupted) {
        if(!rollerSubsystem.algaeDetected()) rollerSubsystem.idle();

        if(Robot.isSimulation()){
            RollerIOInputs newInputs = new RollerIOInputs();
            newInputs.algaeDetected = true;
            newInputs.coralDetected = false;
            rollerSubsystem.overrideSimStates(newInputs);
        }
    }
}
