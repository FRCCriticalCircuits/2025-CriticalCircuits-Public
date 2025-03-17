package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;

public class HomeWrist extends Command {
  private Timer nt = new Timer();

  private ElevatorSubsystem elevatorSubsystem;
  public HomeWrist(ElevatorSubsystem e) {
    this.elevatorSubsystem = e;
    this.addRequirements(e);
    
    
  }

  @Override
  public void initialize() {
    nt.reset();
    nt.start();
    elevatorSubsystem.disableWrist();

    AutoAimManager.getInstance().updateMode(Mode.CORAL_PLACE);
    AutoAimManager.getInstance().updateLevel(Level.L1);

    elevatorSubsystem.setWristVoltage(1);
  }

  @Override
  public boolean isFinished() {
    if (nt.hasElapsed(0.5)) {
      return elevatorSubsystem.getWrist().getVelocity() < 0.05;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setWristVoltage(0);
    elevatorSubsystem.enableWrist();

    elevatorSubsystem.getWrist().resetEncoder();
  }
}
