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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // Game Pieces Publisher
  StructArrayPublisher<Pose3d> coralPublisher, algaePublisher;

  public Robot() {
    m_robotContainer = new RobotContainer();

    coralPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/simulationGamePieces/coral", Pose3d.struct).publish();
    algaePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/simulationGamePieces/algae", Pose3d.struct).publish();
  }

  @Override
  public void robotInit() {
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    if(Robot.isSimulation()) m_robotContainer.resetSimulationField();
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

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
