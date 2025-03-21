// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.KeyBinding;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.WaitElevator;
import frc.robot.commands.climber.WinchDownCommand;
import frc.robot.commands.climber.WinchUpCommand;
import frc.robot.commands.swerve.AutoAlignCommand;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.climber.WinchSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.RollerSubsystem;
import frc.robot.subsystems.elevatoreffector.ElevatorIO;
import frc.robot.subsystems.elevatoreffector.ElevatorIOTalonFX;
import frc.robot.subsystems.elevatoreffector.ElevatorSubsystem2;
import frc.robot.subsystems.elevatoreffector.WristIO;
import frc.robot.subsystems.elevatoreffector.WristIOTalonFX;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;
import org.ironmaple.simulation.SimulatedArena;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

public class RobotContainer {
    private static final CommandXboxController driveController = new CommandXboxController(0);
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final ElevatorSubsystem2 elevatorSubsystem;
    private final RollerSubsystem rollerSubsystem;

    private final LEDSubsystem ledSubsystem;
    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private final Controller controller = Controller.getInstance();
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final AutoAimManager autoAimManager = AutoAimManager.getInstance(
            () -> controller.getDriverLT(),
            () -> controller.getDriverRT());
    private final Robot robot;
    private WinchSubsystem winchSubsystem = WinchSubsystem.getInstance();
    private SendableChooser<Command> ac;

    // private int mode = 1;

    public RobotContainer(Robot r) {
        this.robot = r;

        LEDSubsystem.start();
        LEDSubsystem.getInstance().setColor(Color.kRed);
        ledSubsystem = LEDSubsystem.getInstance();

        swerveSubsystem = SwerveSubsystem.getInstance();
        visionSubsystem = new VisionSubsystem(swerveSubsystem);
        if (Robot.isReal()) {
            elevatorSubsystem = new ElevatorSubsystem2(new ElevatorIOTalonFX(), new WristIOTalonFX());
        } else {
            // Simulation classes
            elevatorSubsystem = new ElevatorSubsystem2(new ElevatorIO() {}, new WristIO() {});
        }
        rollerSubsystem = new RollerSubsystem(robot, elevatorSubsystem);
//		elevatorSubsystem = new ElevatorSubsystem(rollerSubsystem);

        winchSubsystem = WinchSubsystem.getInstance();

        visionSubsystem.start();

        swerveSubsystem.setDefaultCommand(
                TeleopDrive.getInstance(
                        () -> -controller.getDriverLY(), // Left-Positive
                        () -> -controller.getDriverLX(), // Forward-Positive
                        () -> -controller.getDriverRX(), // CCW Positive
                        () -> controller.getDriverLT(),
                        () -> controller.getDriverRT()));


        autoChooser.setDefaultOption("1 Piece Mid", "1 Piece Mid");
        autoChooser.addOption("2 Piece Mid", "1 Piece + 1 Mid");
        autoChooser.addOption("(C) 2 Piece Mid", "(C) 1 Piece + 1 Mid");
        autoChooser.addOption("Taxi", "Taxi");
        // autoChooser.addOption("(Left) 1 Piece Mid", "(Left) 1 Piece Mid");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    public static CommandXboxController getDriveController() {
        return driveController;
    }

    private void configureBindings() {
        // funny experiment
        new Trigger(RobotController::getUserButton).onTrue(new InstantCommand(
                () -> {
                    // play a frequency
                    winchSubsystem.blink(1244.51);
                }
        ).withTimeout(200).andThen(new InstantCommand(() -> {
            winchSubsystem.blink(0);
        })));

        // region drivecontroller
        /*
         * Gyro Reset
         */
        driveController.button(KeyBinding.GYRO_RESET).debounce(0.02).onTrue(
                new InstantCommand(
                        () -> {
                            swerveSubsystem.resetGyro(-0.5);
                        }));

//    operatorController.button(KeyBinding.GYRO_RESET).debounce(0.02).onTrue(
//      new ZeroWrist(elevatorSubsystem)
//    );

        // AUTOAIM
        Supplier<Pose2d> targetPoseSupplier = () -> {
            return autoAimManager.getNearestReef(swerveSubsystem.getPoseEstimate());
        };
        driveController.a().whileTrue(
                new AutoAlignCommand(targetPoseSupplier, swerveSubsystem)
        );

        // Test side step
        // driveController.povRight().debounce(0.02).whileTrue(
        // new RelativeDrive(0.1)
        // );

        // driveController.povLeft().debounce(0.02).whileTrue(
        // new RelativeDrive(-0.1)
        // );

        // endregion drivecontroller

        /*
         * AutoAim Settings
         */
        operatorController.a().debounce(0.02).onTrue(
                elevatorSubsystem.setLevelCommand(Level.L1)
        );

        operatorController.b().debounce(0.02).onTrue(
                elevatorSubsystem.setLevelCommand(Level.L2)
        );

        operatorController.x().debounce(0.02).onTrue(
                elevatorSubsystem.setLevelCommand(Level.L3)
        );

        operatorController.y().debounce(0.02).onTrue(
                elevatorSubsystem.setLevelCommand(Level.L4)
        );

        operatorController.leftBumper().debounce(0.02).onTrue(
                new InstantCommand(
                        () -> {
                            // If algae is present, force mode to always be algae mode
              /* TODO: Fix algae detection
              else if (rollerSubsystem.algaeDetected()) {

                autoAimManager.updateMode(Mode.ALGAE_INTAKE);
              }

              */

                            // Get current mode
                            int mode = elevatorSubsystem.getMode().value;
                            // down-shift mode
                            mode--;
                            if (mode < 0) mode = 2;
                            elevatorSubsystem.setMode(Mode.valueOf(mode));
                        }, elevatorSubsystem, rollerSubsystem));

        operatorController.rightBumper().debounce(0.02).onTrue(
                new InstantCommand(
                        () -> {
                            // get current mode
                            int mode = elevatorSubsystem.getMode().value;
                            // up-shift mode
                            mode++;
                            if (mode > 2) mode = 0;
                            elevatorSubsystem.setMode(Mode.valueOf(mode));
                        }, elevatorSubsystem, rollerSubsystem));

//        operatorController.povUp().debounce(0.02).whileTrue(
//                new WinchUpCommand(winchSubsystem, rollerSubsystem, elevatorSubsystem));
//
//        operatorController.povDown().debounce(0.02).whileTrue(
//                new WinchDownCommand(winchSubsystem, rollerSubsystem, elevatorSubsystem));

        driveController.leftBumper().debounce(0.02).whileTrue(
                new ParallelCommandGroup(
                        elevatorSubsystem.fetchAlgaeCommand(),
                        rollerSubsystem.intakeAlgaeCommand()
                )
        );

        driveController.rightBumper().debounce(0.02).whileTrue(
                rollerSubsystem.intakeCoralCommand()
        );

        driveController.y().debounce(0.02).whileTrue(
                rollerSubsystem.outtakeCoralCommand(elevatorSubsystem::getLevel));

        NamedCommands.registerCommand(
                "setModeCoral",
                elevatorSubsystem.setModeCommand(Mode.CORAL_PLACE));

        NamedCommands.registerCommand(
                "setModeIntake",
                elevatorSubsystem.setModeCommand(Mode.CORAL_INTAKE));

        NamedCommands.registerCommand(
                "setModeAlgae",
                elevatorSubsystem.setModeCommand(Mode.ALGAE_INTAKE));

        NamedCommands.registerCommand(
                "setL1",
                elevatorSubsystem.setLevelCommand(Level.L1));

        NamedCommands.registerCommand(
                "setL2",
                elevatorSubsystem.setLevelCommand(Level.L2));

        NamedCommands.registerCommand(
                "setL3",
                elevatorSubsystem.setLevelCommand(Level.L3));

        NamedCommands.registerCommand(
                "setL4",
                elevatorSubsystem.setLevelCommand(Level.L4));

        NamedCommands.registerCommand(
                "waitElevator",
                new WaitElevator(elevatorSubsystem));

        NamedCommands.registerCommand(
                "outTake",
                rollerSubsystem.outtakeCoralCommand(elevatorSubsystem::getLevel));

        NamedCommands.registerCommand(
                "intakeCoral",
                rollerSubsystem.intakeCoralCommand().until(rollerSubsystem::hasCoral));
        NamedCommands.registerCommand(
                "intakeAlgae",
                rollerSubsystem.intakeAlgaeCommand().until(rollerSubsystem::hasAlgae));
        NamedCommands.registerCommand(
                "fetchAlgae",
                elevatorSubsystem.fetchAlgaeCommand().until(rollerSubsystem::hasAlgae));

        // Vision commands
        NamedCommands.registerCommand(
                "visionEnable",
                new InstantCommand(
                        this.visionSubsystem::enable));

        NamedCommands.registerCommand(
                "visionDisable",
                new InstantCommand(
                        this.visionSubsystem::disable));
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
}
