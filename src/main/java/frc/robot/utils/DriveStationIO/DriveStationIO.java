package frc.robot.utils.DriveStationIO;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriveStationIO {
    private DriveStationIO(){}

    /**
     * check if the robot is in Blue Alliance
     * @return true if <b>Blue</b>, false if <b>Red</b> or <b>neither</b>
     */
    public static boolean isBlue(){
        return (DriverStation.getAlliance().isPresent()) ? DriverStation.getAlliance().get() == Alliance.Blue : false; 
    }

    /**
     * check if the robot is in Red Alliance
     * @return true if <b>Red</b>, false if <b>Blue</b> or <b>neither</b>
     */
    public static boolean isRed(){
        return (DriverStation.getAlliance().isPresent()) ? DriverStation.getAlliance().get() == Alliance.Red : false; 
    }

    /**
     * get Alliance selected
     * @param defaultResult the value returned if the Alliance is not present
     * @return Alliance Type in {@link Alliance}
     */
    public static Alliance getAlliance(Alliance defaultResult){
        return (DriverStation.getAlliance().isPresent()) ? DriverStation.getAlliance().get() : defaultResult;
    }

    /**
     * get Alliance selected
     * @return Alliance Type, Blue Alliance as default result
     */
    public static Alliance getAlliance(){
        return getAlliance(Alliance.Blue);
    }

    public static boolean isAuto(){
        return DriverStation.isAutonomous();
    }

    public static boolean isAutoEnabled(){
        return DriverStation.isAutonomousEnabled();
    }

    public static boolean isTeleop(){
        return DriverStation.isTeleop();
    }

    public static boolean isTeleopEnabled(){
        return DriverStation.isTeleopEnabled();
    }

    public static boolean isEnabled(){
        return DriverStation.isEnabled();
    }

    public static boolean isDisabled(){
        return DriverStation.isDisabled();
    }
}

