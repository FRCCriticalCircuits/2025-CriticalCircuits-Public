// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.KeyBinding;
import frc.robot.commands.teleopDrive;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;


public class RobotContainer {
  private SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private VisionSubsystem visionSubsystem = new VisionSubsystem();

  private SendableChooser<String> autoChooser = new SendableChooser<>();

  private Controller controller = Controller.getInstance();
  private CommandXboxController driveController = new CommandXboxController(0); 

  private AutoAimManager autoAimManager = AutoAimManager.getInstance(
    () -> controller.getDriverLT(),
    () -> controller.getDriverRT()
  );

  public RobotContainer() {
    visionSubsystem.start();
    
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
    autoChooser.addOption("test", "Auto 1");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    driveController.button(KeyBinding.GYRO_RESET).debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          swerveSubsystem.resetGyro(0);
        }
      )
    );    

    driveController.a().debounce(0.02).onTrue(
      new InstantCommand(
        ()  -> {
          new SequentialCommandGroup(
            new InstantCommand(
              () -> {
                teleopDrive.manualEnable = false;
              }, swerveSubsystem
            ),
            new ParallelDeadlineGroup(
              new WaitCommand(3),
              autoAimManager.getCommand()
            ),
            new InstantCommand(
              () -> {
                teleopDrive.manualEnable = true;
              }, swerveSubsystem
            )
          ).schedule();
        }, swerveSubsystem
      )
    );

    driveController.x().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          autoAimManager.cancle();
          teleopDrive.manualEnable = true;
        }, swerveSubsystem
      )
    );
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new InstantCommand(
        () -> {
          Optional<Pose2d> initialPose;
          try {
            initialPose = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected()).get(0).getStartingHolonomicPose();
          } catch (IOException | ParseException e) {
            throw new RuntimeException("[Pathplanner] No auto file found");
          }

          if(initialPose.isEmpty()) {
            throw new RuntimeException("[Pathplanner] No starting pose found in auto file");
          }else{
            swerveSubsystem.resetPoseEstimate(
              initialPose.get()
            );
          }
        }, swerveSubsystem
      ),
      AutoBuilder.buildAuto(autoChooser.getSelected())
    );
  }
}
