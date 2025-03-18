package frc.robot.commands.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.elevator.ArmIO;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;

public class ZeroWrist extends Command {
  private Timer nt = new Timer();
  private ArmIO wrist;

  private ElevatorSubsystem elevatorSubsystem;
  public ZeroWrist(ElevatorSubsystem e) {
    this.elevatorSubsystem = e;
    this.wrist = elevatorSubsystem.getWrist();

    this.addRequirements(e);
  }

  @Override
  public void initialize() {
    nt.reset();
    nt.start();
    // take control of wrist (disable subsystem-local)
    wrist.disable();

    AutoAimManager.getInstance().updateMode(Mode.CORAL_PLACE);
    AutoAimManager.getInstance().updateLevel(Level.L1);

    // Run wrist backwards until hard stop is hit
    wrist.setVoltage(WRIST_HOMING_VOLTAGE);
  }

  @Override
  public boolean isFinished() {
    if (nt.hasElapsed(0.5)) {
      // detect hard stop by stall condition
      return elevatorSubsystem.getWrist().getVelocity() < 0.05;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    wrist.setVoltage(0);
    wrist.resetEncoder();
    wrist.enable();
  }
}
