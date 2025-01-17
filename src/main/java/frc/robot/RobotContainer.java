// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerBinding;
import frc.robot.commands.teleopDrive;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.autoaim.AutoAimManager;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {
  private SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private SendableChooser<String> autoChooser = new SendableChooser<>();

  private Controller controller = Controller.getInstance();
  private CommandXboxController driveController = new CommandXboxController(0); 

  Command autoAimCommandGroup = new SequentialCommandGroup
  (
    AutoAimManager.getInstance().runSwerveAutoAim(
      () -> swerveSubsystem.getPoseEstimate().getRotation().getDegrees(),
      new Translation2d(
        0,
        0
      )
    )
  );

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
      teleopDrive.getInstance(
        () -> -controller.getDriverLY(),    // Left-Positive
        () -> -controller.getDriverLX(),    // Forward-Positive
        () -> -controller.getDriverRX(),    // CCW Positive
        () -> controller.getDriverLT(),
        () -> controller.getDriverRT()
      )
    );

    autoChooser.setDefaultOption("1", "Auto 0");
    autoChooser.addOption("2", "Auto 1");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    driveController.button(ControllerBinding.GYRO_RESET).debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          swerveSubsystem.resetGyro(0);
        }
      )
    );    

    driveController.a().debounce(0.02).onTrue(
      autoAimCommandGroup
    );

    driveController.x().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          CommandScheduler.getInstance().cancel(autoAimCommandGroup);
        }
      )
    );

    driveController.a().debounce(0.02).onChange(
      new InstantCommand(
        () -> {
          teleopDrive.manualEnable = !teleopDrive.manualEnable;
        }
      )
    );
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new InstantCommand(
        () -> {
          try {
            swerveSubsystem.resetPoseEstimate(
              PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected())
                             .get(0)
                             .getStartingHolonomicPose().get() // Should be Holonomic but we don't need starting velocity... etc
            );
          } catch (IOException | ParseException e) {
            e.printStackTrace();
          }
        }, swerveSubsystem
      ),
      AutoBuilder.buildAuto(autoChooser.getSelected())
    );
  }
}
