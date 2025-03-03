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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.KeyBinding;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.Shoot;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.WaitElevator;
import frc.robot.commands.climber.WinchDownCommand;
import frc.robot.commands.climber.WinchUpCommand;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.climber.WinchSubsystem;
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

  private ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
  private WinchSubsystem winchSubsystem = WinchSubsystem.getInstance();

  @SuppressWarnings("unused")
  private RollerSubsystem rollerSubsystem = RollerSubsystem.getInstance();

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
    visionSubsystem.start();
    
    swerveSubsystem.setDefaultCommand(
      TeleopDrive.getInstance(
        () -> -controller.getDriverLY(),    // Left-Positive
        () -> -controller.getDriverLX(),    // Forward-Positive
        () -> -controller.getDriverRX(),    // CCW Positive
        () -> controller.getDriverLT(),
        () -> controller.getDriverRT()
      )
    );

    autoChooser.setDefaultOption("Upper Coral", "Auto 0");
    autoChooser.addOption("testing", "armElevTest");

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
          TeleopDrive.manualEnable = true;
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
          if(mode == 2) elevatorSubsystem.fetchAlgae = false;
        }, elevatorSubsystem, rollerSubsystem
      )
    );

    operatorController.rightBumper().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
          mode++;
          if(mode > 2) mode = 0;
          autoAimManager.updateMode(Mode.valueOf(mode));
          if(mode == 2) elevatorSubsystem.fetchAlgae = false;
        }, elevatorSubsystem, rollerSubsystem
      )
    );

    operatorController.povUp().debounce(0.02).whileTrue(
        new WinchUpCommand(winchSubsystem)
    );

    operatorController.povDown().debounce(0.02).whileTrue(
        new WinchDownCommand(winchSubsystem)
    );

    driveController.leftBumper().debounce(0.02).onTrue(
      new InstantCommand(
        () -> {
            elevatorSubsystem.fetchAlgae = !elevatorSubsystem.fetchAlgae;
        }, elevatorSubsystem, rollerSubsystem
      )
    );

    driveController.rightBumper().debounce(0.02).whileTrue(
      new IntakeAlgae()
    );

    driveController.b().debounce(0.02).whileTrue(
      new IntakeCoral()
    );

    driveController.y().debounce(0.02).whileTrue(
      new Shoot()
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
      "setModeAlgae", 
      new InstantCommand(
        () -> {
          autoAimManager.updateMode(Mode.ALGAE_PICK);
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
      "setL2",
      new InstantCommand(
        () -> {
          autoAimManager.updateLevel(Level.L2);
        }
      )
    );

    NamedCommands.registerCommand(
      "setL3",
      new InstantCommand(
        () -> {
          autoAimManager.updateLevel(Level.L3);
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
      new WaitElevator()
    );

    NamedCommands.registerCommand(
      "outTake",
      new Shoot().withTimeout(1).withDeadline(
        new WaitUntilCommand(
          () -> !rollerSubsystem.algaeDetected() &&
                !rollerSubsystem.coralDetected()
        )
      )
    );

    NamedCommands.registerCommand(
      "intakeCoral",
      new IntakeCoral().withTimeout(1.2).withDeadline(
        new WaitUntilCommand(
          () -> rollerSubsystem.coralDetected()
        )
      )
    );

    NamedCommands.registerCommand(
      "intakeAlgae",
      new IntakeAlgae().withTimeout(1.2).withDeadline(
        new WaitUntilCommand(
          () -> rollerSubsystem.algaeDetected()
        )
      )
    );

    NamedCommands.registerCommand(
      "fetchAlgae", 
      new InstantCommand(
        () -> {
            elevatorSubsystem.fetchAlgae = !elevatorSubsystem.fetchAlgae;
        }, elevatorSubsystem
      )
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
