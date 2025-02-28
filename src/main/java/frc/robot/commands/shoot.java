package frc.robot.commands;


import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.RollerSubsystem;

public class shoot extends Command{
    private RollerSubsystem rollerSubsystem;
    private ElevatorSubsystem elevatorSubsystem;

    private double timeEnds, timeLimitSeconds;
    
    public shoot(double timeLimitSeconds){
        rollerSubsystem = RollerSubsystem.getInstance();
        elevatorSubsystem = ElevatorSubsystem.getInstance();

        this.timeLimitSeconds = timeLimitSeconds;

        addRequirements(rollerSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(SwerveSubsystem.getInstance());
    }

    public void initialize(){
        this.timeEnds = Timer.getFPGATimestamp() + timeLimitSeconds;
        rollerSubsystem.outTake();
        
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
                        MetersPerSecond.of(2.5),
                        wristTranslation.getRotation().getMeasureY().times(-1)
                    ).enableBecomesGamePieceOnFieldAfterTouchGround()
                );
            }   
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(rollerSubsystem.coralDetected() || rollerSubsystem.algaeDetected()) rollerSubsystem.hold();
        else rollerSubsystem.idle();

        if(Robot.isSimulation()){
            RollerIOInputs newInputs = new RollerIOInputs();
            newInputs.algaeDetected = false;
            newInputs.coralDetected = false;
            rollerSubsystem.overrideSimStates(newInputs);
        }
    }

    @Override
    public boolean isFinished() {
        return (!rollerSubsystem.coralDetected() && !rollerSubsystem.algaeDetected()) || (Timer.getFPGATimestamp() > timeEnds);
    }
}
