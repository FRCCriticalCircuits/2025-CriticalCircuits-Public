package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class waitElevator extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    
    public waitElevator(){
        elevatorSubsystem = ElevatorSubsystem.getInstance();
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        Timer.delay(0.1);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.curState == elevatorSubsystem.targetState;
    }
}
