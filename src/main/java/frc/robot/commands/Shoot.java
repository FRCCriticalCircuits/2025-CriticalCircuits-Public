package frc.robot.commands;


import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.subsystems.elevator.RollerIO.RollerMode;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.RollerSubsystem;

public class Shoot extends Command{
    private RollerSubsystem rollerSubsystem;
    private ElevatorSubsystem elevatorSubsystem;

    public Shoot(){
        rollerSubsystem = RollerSubsystem.getInstance();
        elevatorSubsystem = ElevatorSubsystem.getInstance();

        addRequirements(rollerSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize(){
        rollerSubsystem.set(RollerMode.OUT);
        
        if(Robot.isSimulation()){
            Pose3d wristTranslation = elevatorSubsystem.getRollerTransltaion();

            if(rollerSubsystem.coralDetected()){
                SimulatedArena.getInstance().addGamePieceProjectile(
                    new ReefscapeCoralOnFly(
                        SwerveSubsystem.driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                        wristTranslation.getTranslation().toTranslation2d(),
                        SwerveSubsystem.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        SwerveSubsystem.driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                        wristTranslation.getMeasureZ(),
                        MetersPerSecond.of(3),
                        wristTranslation.getRotation().getMeasureY().times(-1)
                    ).enableBecomesGamePieceOnFieldAfterTouchGround()
                );
            } 
            
            if(rollerSubsystem.algaeDetected()){
                SimulatedArena.getInstance().addGamePieceProjectile(
                    new ReefscapeAlgaeOnFly(
                        SwerveSubsystem.driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                        wristTranslation.getTranslation().toTranslation2d(),
                        SwerveSubsystem.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        SwerveSubsystem.driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                        wristTranslation.getMeasureZ(),
                        MetersPerSecond.of(3),
                        wristTranslation.getRotation().getMeasureY().times(-1)
                    ).enableBecomesGamePieceOnFieldAfterTouchGround()
                );
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!rollerSubsystem.coralDetected() && !rollerSubsystem.algaeDetected()) rollerSubsystem.set(RollerMode.IDLE);
        else rollerSubsystem.set(RollerMode.HOLD);

        if(Robot.isSimulation()){
            RollerIOInputs newInputs = new RollerIOInputs();
            newInputs.algaeDetected = false;
            newInputs.coralDetected = false;
            rollerSubsystem.overrideSimStates(newInputs);
        }
    }
}
