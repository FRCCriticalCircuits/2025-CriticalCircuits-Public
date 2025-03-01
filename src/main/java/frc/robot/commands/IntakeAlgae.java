package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerSubsystem;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;

public class IntakeAlgae extends Command{
    private RollerSubsystem rollerSubsystem;
    private double timeEnds, timeLimitSeconds;
    
    public IntakeAlgae(double timeLimitSeconds){
        rollerSubsystem = RollerSubsystem.getInstance();
        this.timeLimitSeconds = timeLimitSeconds;

        addRequirements(rollerSubsystem);
    }

    public void initialize(){
        this.timeEnds = Timer.getFPGATimestamp() + timeLimitSeconds;
        rollerSubsystem.intake();
    }

    @Override
    public void end(boolean interrupted) {
        if(rollerSubsystem.algaeDetected()) rollerSubsystem.hold();
        else rollerSubsystem.idle();

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
