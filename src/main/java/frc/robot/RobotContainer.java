// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.KeyBinding;
import frc.robot.commands.intakeCoral;
import frc.robot.commands.shoot;
import frc.robot.commands.teleopDrive;
import frc.robot.commands.waitElevator;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.RollerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;
import frc.robot.utils.structures.DataStrcutures.Spot;

public class RobotContainer {
  private SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private VisionSubsystem visionSubsystem = new VisionSubsystem();

  @SuppressWarnings("unused")
  private ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();

  @SuppressWarnings("unused")
  private RollerSubsystem rollerSubsystem = new RollerSubsystem();

  private SendableChooser<String> autoChooser = new SendableChooser<>();

  private Controller controller = Controller.getInstance();
  private CommandXboxController driveController = new CommandXboxController(0); 
  private CommandXboxController operatorController = new CommandXboxController(1);

  private AutoAimManager autoAimManager = AutoAimManager.getInstance(
    () -> controller.getDriverLT(),
    () -> controller.getDriverRT()
  );

  private int mode = 1;

  public RobotContainer() {
    // visionSubsystem.start();
    
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

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    /*
     * Gyro Reset
     */
    driveController.button(KeyBinding.GYRO_RESET).debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          swerveSubsystem.resetGyro(-0.5);
        }
      )
    ); 

    /*
     * AutoAim
     */   
    driveController.a().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          new ParallelDeadlineGroup(
            new WaitCommand(2.5),
            autoAimManager.getCommand()
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

    /*
     * AutoAim Settings
     */
    operatorController.a().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          autoAimManager.updateLevel(Level.L1);
          autoAimManager.updateSpot(Spot.MID);
        }
      )
    );

    operatorController.b().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          autoAimManager.updateLevel(Level.L2);
        }
      )
    );

    operatorController.x().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          autoAimManager.updateLevel(Level.L3);
        }
      )
    );

    operatorController.y().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          autoAimManager.updateLevel(Level.L4);
        }
      )
    );

    operatorController.povLeft().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          autoAimManager.updateSpot(Spot.L);    
        }
      )
    );

    operatorController.povRight().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          autoAimManager.updateSpot(Spot.R);    
        }
      )
    );

    operatorController.leftBumper().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          mode--;
          if(mode < 0) mode = 2;
          autoAimManager.updateMode(Mode.valueOf(mode));
        }
      )
    );

    operatorController.rightBumper().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          mode++;
          if(mode > 2) mode = 0;
          autoAimManager.updateMode(Mode.valueOf(mode));
        }
      )
    );

    driveController.b().debounce(0.02).onTrue(
      new intakeCoral(1.5)
    );

    driveController.y().debounce(0.02).onTrue(
      new shoot(0.7)
    );
  }

  public Command getAutonomousCommand() {
    registerCommands();
    
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

  public void registerCommands(){
    NamedCommands.registerCommand(
      "setModeCoral",
      new InstantCommand(
        () -> {
          autoAimManager.updateMode(Mode.CORAL_PLACE);
        }
      )
    );

    NamedCommands.registerCommand(
      "setModeIntake",
      new InstantCommand(
        () -> {
          autoAimManager.updateMode(Mode.CORAL_INTAKE);
        }
      )
    );

    NamedCommands.registerCommand(
      "setL1",
      new InstantCommand(
        () -> {
          autoAimManager.updateLevel(Level.L1);
        }
      )
    );

    NamedCommands.registerCommand(
      "setL4",
      new InstantCommand(
        () -> {
          autoAimManager.updateLevel(Level.L4);
        }
      )
    );

    NamedCommands.registerCommand(
      "setLeft",
      new InstantCommand(
        () -> {
          autoAimManager.updateSpot(Spot.L);
        }
      )
    );

    NamedCommands.registerCommand(
      "setRight",
      new InstantCommand(
        () -> {
          autoAimManager.updateSpot(Spot.R);
        }
      )
    );

    NamedCommands.registerCommand(
      "waitElevator",
      new waitElevator()
    );

    NamedCommands.registerCommand(
      "outTake",
      new shoot(0.7)
    );

    NamedCommands.registerCommand(
      "intakeCoral",
      new intakeCoral(1.4)
    );
  }

  public void resetSimulationField() {
    swerveSubsystem.resetPoseEstimate
    (
      DriveStationIO.isBlue() ? FieldConstants.INIT_POSE_BLUE 
                              : FieldConstants.INIT_POSE_BLUE.flip()
    );
    
    swerveSubsystem.resetGyro(-0.5);
    SimulatedArena.getInstance().resetFieldForAuto();
  }
  
  public void updateSimulation() {
    SimulatedArena.getInstance().simulationPeriodic();
  }
}
