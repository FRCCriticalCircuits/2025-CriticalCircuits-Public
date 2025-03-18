// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // Game Pieces Publisher
  StructArrayPublisher<Pose3d> coralPublisher, algaePublisher;

  private Timer m_gcTimer;

  public Robot() {
    // AdvantageKit Logging
    Logger.recordMetadata("ProjectName", "9062-REEFSCAPE-RIPTIDE"); // Set a metadata value

    if (isReal()) { // Only log to usb if robot is real
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    m_gcTimer = new Timer();
    m_gcTimer.start();
    m_robotContainer = new RobotContainer(this);

    coralPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/simulationGamePieces/coral", Pose3d.struct).publish();
    algaePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/simulationGamePieces/algae", Pose3d.struct).publish();
  }

  @Override
  public void robotInit() {
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Garbage collect more often
    if (m_gcTimer.advanceIfElapsed(5)) {
     System.gc();
    }

  }

  @Override
  public void disabledInit() {
    if(Robot.isSimulation()) m_robotContainer.resetSimulationField();

    LEDSubsystem.getInstance().setColor(Color.kRed);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    LEDSubsystem.getInstance().setColor(Color.kRed);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    SignalLogger.start();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
      m_robotContainer.updateSimulation();

      algaePublisher.set(SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
      coralPublisher.set(SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
  }
}
