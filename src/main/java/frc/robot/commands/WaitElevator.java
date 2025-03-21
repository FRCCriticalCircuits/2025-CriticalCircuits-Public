package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevatoreffector.ElevatorSubsystem2;

public class WaitElevator extends Command{
    private ElevatorSubsystem2 elevatorSubsystem;
    
    public WaitElevator(ElevatorSubsystem2 elev){
        elevatorSubsystem = elev;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isAtTargetState();
    }
}
