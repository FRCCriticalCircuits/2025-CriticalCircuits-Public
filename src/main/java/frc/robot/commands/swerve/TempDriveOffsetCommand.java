package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TeleopDrive;

public class TempDriveOffsetCommand extends Command {
  private double x, y;
  private int MAX_TIME = 1;
  public TempDriveOffsetCommand(double x, double y) {
    this.x = x;
    this.y = y;
  }

  @Override
  public void initialize() {
    TeleopDrive.setRelativeXSpeedOffset(x);
    TeleopDrive.setRelativeYSpeedOffset(y);

    // Force the command to cancel if run for too long to prevent loss of driver control
    new WaitCommand(MAX_TIME).andThen(new InstantCommand(() -> {
      cancel();
    })).schedule();
  }

  @Override
  public void end(boolean interrupted) {
    TeleopDrive.setRelativeXSpeedOffset(0);
    TeleopDrive.setRelativeYSpeedOffset(0);
  }
}
