// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.AutoAimConstants.REEF_POSES_BLUE;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.KeyBinding;
import frc.robot.commands.climber.WinchDownCommand;
import frc.robot.commands.climber.WinchUpCommand;
import frc.robot.commands.swerve.AutoAlignCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.AutoAimManager.Reef;
import frc.robot.subsystems.climber.WinchSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.RollerSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;
import frc.robot.utils.structures.DataStrcutures.Spot;

public class RobotContainer {
	private Robot robot;
	private SwerveSubsystem swerveSubsystem;
	private VisionSubsystem visionSubsystem;

	private ElevatorSubsystem elevatorSubsystem;
	private RollerSubsystem rollerSubsystem;

	private LEDSubsystem ledSubsystem;

	private WinchSubsystem winchSubsystem = WinchSubsystem.getInstance();

	private SendableChooser<String> autoChooser = new SendableChooser<>();
	private SendableChooser<String> ac = new SendableChooser<>();

	private Controller controller = Controller.getInstance();

	private static CommandXboxController driveController = new CommandXboxController(0);
	private CommandXboxController operatorController = new CommandXboxController(1);

	private AutoAimManager autoAimManager = AutoAimManager.getInstance(
			() -> controller.getDriverLT(),
			() -> controller.getDriverRT());

	// private int mode = 1;

	public RobotContainer(Robot r) {
		this.robot = r;

		LEDSubsystem.start();
		LEDSubsystem.getInstance().setColor(Color.kRed);

		swerveSubsystem = SwerveSubsystem.getInstance();
		visionSubsystem = new VisionSubsystem(swerveSubsystem);
		rollerSubsystem = new RollerSubsystem(robot);
		elevatorSubsystem = new ElevatorSubsystem(rollerSubsystem);
		ledSubsystem = LEDSubsystem.getInstance();
		winchSubsystem = WinchSubsystem.getInstance();

		visionSubsystem.start();

		swerveSubsystem.setDefaultCommand(
				TeleopDrive.getInstance(
						() -> -controller.getDriverLY(), // Left-Positive
						() -> -controller.getDriverLX(), // Forward-Positive
						() -> -controller.getDriverRX(), // CCW Positive
						() -> controller.getDriverLT(),
						() -> controller.getDriverRT()));

		// ac = AutoBuilder.buildAutoChooser();
		// ac.setDefaultOption("1 Piece Mid", "1 Piece Mid");

		autoChooser.setDefaultOption("1 Piece Mid", "1 Piece Mid");
		autoChooser.addOption("test2", "test2");
		autoChooser.addOption("dist", "dtest");

		SmartDashboard.putData("Auto Chooser", autoChooser);

		configureBindings();
	}

	private void configureBindings() {

		// region drivecontroller
		/*
		 * Gyro Reset
		 */
		driveController.button(KeyBinding.GYRO_RESET).debounce(0.02).onTrue(
				new InstantCommand(
						() -> {
							swerveSubsystem.resetGyro(-0.5);
						}));

		/*
		 * AutoAim
		 */
		// driveController.a().debounce(0.02).onTrue(
		// 		new InstantCommand(
		// 				() -> {
		// 					new ParallelDeadlineGroup(
		// 							new WaitCommand(2.5),
		// 							autoAimManager.pathFindCommand()).schedule();
		// 				}, swerveSubsystem));
		  Supplier<Pose2d> targetPoseSupplier = ()->{return autoAimManager.getNearestReef(swerveSubsystem.getPoseEstimate(),Reef.LEFT);};
          driveController.a().whileTrue(
            new AutoAlignCommand(targetPoseSupplier, swerveSubsystem)
            );

		// driveController.povRight().debounce(0.02).whileTrue(
		// new RelativeDrive(0.1)
		// );

		// driveController.povLeft().debounce(0.02).whileTrue(
		// new RelativeDrive(-0.1)
		// );

		driveController.x().debounce(0.02).onTrue(
				new InstantCommand(
						() -> {
							autoAimManager.cancle();
							TeleopDrive.manualEnable = true;
						}, swerveSubsystem));

		// endregion drivecontroller

		/*
		 * AutoAim Settings
		 */
		operatorController.a().debounce(0.02).onTrue(
				new InstantCommand(
						() -> {
							autoAimManager.updateLevel(Level.L1);
							autoAimManager.updateSpot(Spot.MID);
						}));

		operatorController.b().debounce(0.02).onTrue(
				new InstantCommand(
						() -> {
							autoAimManager.updateLevel(Level.L2);
						}));

		operatorController.x().debounce(0.02).onTrue(
				new InstantCommand(
						() -> {
							autoAimManager.updateLevel(Level.L3);
						}));

		operatorController.y().debounce(0.02).onTrue(
				new InstantCommand(
						() -> {
							autoAimManager.updateLevel(Level.L4);
						}));

		operatorController.povLeft().debounce(0.02).onTrue(
				new InstantCommand(
						() -> {
							autoAimManager.updateSpot(Spot.L);
						}));

		operatorController.povRight().debounce(0.02).onTrue(
				new InstantCommand(
						() -> {
							autoAimManager.updateSpot(Spot.R);
						}));

		operatorController.leftBumper().debounce(0.02).onTrue(
				new InstantCommand(
						() -> {
              // If coral is present, force mode to always be coral placement
              if (rollerSubsystem.hasCoral()) {
                autoAimManager.updateMode(Mode.CORAL_PLACE);
                return;
              } 
              // If algae is present, force mode to always be algae mode
              /* TODO: Fix algae detection
              else if (rollerSubsystem.algaeDetected()) {
                
                autoAimManager.updateMode(Mode.ALGAE_PICK);
              }

              */

							// Get current mode
							int mode = autoAimManager.getMode().value;
							// down-shift mode
							mode--;
							if (mode < 0) mode = 2;
							autoAimManager.updateMode(Mode.valueOf(mode));
							// unknown override no clue why this is here
              if (mode == 2) elevatorSubsystem.fetchAlgae = false;
						}, elevatorSubsystem, rollerSubsystem));

		operatorController.rightBumper().debounce(0.02).onTrue(
				new InstantCommand(
						() -> {
              // get current mode
							int mode = autoAimManager.getMode().value;
							// up-shift mode
              mode++;
							if (mode > 2) mode = 0;
							autoAimManager.updateMode(Mode.valueOf(mode));
              // unknown override no clue why this is here
							if (mode == 2) elevatorSubsystem.fetchAlgae = false;
						}, elevatorSubsystem, rollerSubsystem));

		operatorController.povUp().debounce(0.02).whileTrue(
				new WinchUpCommand(winchSubsystem, rollerSubsystem));

		operatorController.povDown().debounce(0.02).whileTrue(
				new WinchDownCommand(winchSubsystem, rollerSubsystem));

		driveController.leftBumper().debounce(0.02).onTrue(
				new InstantCommand(
						() -> {
							elevatorSubsystem.fetchAlgae = !elevatorSubsystem.fetchAlgae;
						}, elevatorSubsystem, rollerSubsystem));

		driveController.rightBumper().debounce(0.02).whileTrue(
				new IntakeAlgae(rollerSubsystem));

		driveController.b().debounce(0.02).whileTrue(
				new IntakeCoral(rollerSubsystem));

		driveController.y().debounce(0.02).whileTrue(
				new Shoot(rollerSubsystem));

		NamedCommands.registerCommand(
				"setModeCoral",
				new InstantCommand(
						() -> {
							autoAimManager.updateMode(Mode.CORAL_PLACE);
							ledSubsystem.setColor(Color.kRed);
						}, ledSubsystem));

		NamedCommands.registerCommand(
				"setModeIntake",
				new InstantCommand(
						() -> {
							autoAimManager.updateMode(Mode.CORAL_INTAKE);
							ledSubsystem.setColor(Color.kBlue);
						}, ledSubsystem));

		NamedCommands.registerCommand(
				"setModeAlgae",
				new InstantCommand(
						() -> {
							autoAimManager.updateMode(Mode.ALGAE_PICK);
							ledSubsystem.setColor(Color.kGreen);
						}, ledSubsystem));

		NamedCommands.registerCommand(
				"setL1",
				new InstantCommand(
						() -> {
							autoAimManager.updateLevel(Level.L1);
						}));

		NamedCommands.registerCommand(
				"setL2",
				new InstantCommand(
						() -> {
							autoAimManager.updateLevel(Level.L2);
						}));

		NamedCommands.registerCommand(
				"setL3",
				new InstantCommand(
						() -> {
							autoAimManager.updateLevel(Level.L3);
						}));

		NamedCommands.registerCommand(
				"setL4",
				new InstantCommand(
						() -> {
							autoAimManager.updateLevel(Level.L4);
						}));

		NamedCommands.registerCommand(
				"setLeft",
				new InstantCommand(
						() -> {
							autoAimManager.updateSpot(Spot.L);
						}));

		NamedCommands.registerCommand(
				"setRight",
				new InstantCommand(
						() -> {
							autoAimManager.updateSpot(Spot.R);
						}));

		NamedCommands.registerCommand(
				"waitElevator",
				new WaitElevator(elevatorSubsystem));

		NamedCommands.registerCommand(
				"outTake",
				new Shoot(rollerSubsystem).withTimeout(1));

		NamedCommands.registerCommand(
				"outTakeL1",
				new SequentialCommandGroup(
						new InstantCommand(
								() -> {
									rollerSubsystem.lowVoltage = true;

								}),
						new Shoot(rollerSubsystem).withTimeout(1),
						new InstantCommand(
								() -> {
									rollerSubsystem.lowVoltage = false;
								})));

		NamedCommands.registerCommand(
				"intakeCoral",
				new AutoIntakeCoral(rollerSubsystem));

		NamedCommands.registerCommand(
				"autoIntakeCoral", new AutoIntakeCoral(rollerSubsystem));

		NamedCommands.registerCommand(
				"intakeAlgae",
				new IntakeAlgae(rollerSubsystem).withTimeout(1.2));

		NamedCommands.registerCommand(
				"fetchAlgae",
				new InstantCommand(
						() -> {
							elevatorSubsystem.fetchAlgae = !elevatorSubsystem.fetchAlgae;
						}, elevatorSubsystem, rollerSubsystem));

		// Vision commands
		NamedCommands.registerCommand(
				"visionEnable",
				new InstantCommand(
						() -> {
							this.visionSubsystem.enable();
						}));

		NamedCommands.registerCommand(
				"visionDisable",
				new InstantCommand(
						() -> {
							this.visionSubsystem.disable();
						}));
	}

	public Command getAutonomousCommand() {
		return new SequentialCommandGroup(
				new InstantCommand(
						() -> {
							Optional<Pose2d> initialPose;

							try {
								initialPose = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected()).get(0)
										.getStartingHolonomicPose();
							} catch (IOException | ParseException e) {
								throw new RuntimeException("[Pathplanner] No auto file found");
							}

							if (initialPose.isEmpty()) {
								throw new RuntimeException("[Pathplanner] No starting pose found in auto file");
							} else {
								swerveSubsystem.resetPoseEstimate(
										initialPose.get());
							}
						}, swerveSubsystem),
				AutoBuilder.buildAuto(autoChooser.getSelected()));
	}

	public void resetSimulationField() {
		swerveSubsystem.resetPoseEstimate(
				DriveStationIO.isBlue() ? FieldConstants.INIT_POSE_BLUE
						: FieldConstants.INIT_POSE_BLUE.flip());

		swerveSubsystem.resetGyro(-0.5);
		SimulatedArena.getInstance().resetFieldForAuto();
	}

	public void updateSimulation() {
		SimulatedArena.getInstance().simulationPeriodic();
	}

	public static CommandXboxController getDriveController() {
		return driveController;
	}
}
