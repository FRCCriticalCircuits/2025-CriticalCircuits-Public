package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DeviceID;

public class Controller extends SubsystemBase{
    public static Controller instance;

    private XboxController driverController;

    private RUMBLE_STATE rumbleState = RUMBLE_STATE.DISABLE;

    public Controller(){
        driverController = new XboxController(DeviceID.GAMEPAD_DRIVER);
    }

    public static Controller getInstance() {
        if (instance == null) {
            instance = new Controller();
        }
        return instance;
    }

    /**
     * Get the instance of Driver's controller
        * @return the {@link XboxController} instance
     */
    public XboxController getDrive(){
        return driverController;
    }

    /**
     * get the X Axis of Driver's Left Joystick
     * @return the value [0,1] in double
     */
    public double getDriverLX(){
        return driverController.getLeftX();
    }

    /**
     * get the Y Axis of Driver's Left Joystick
     * @return the value [0,1] in double
     */
    public double getDriverLY(){
        return driverController.getLeftY();
    }
    
    /**
     * get the X Axis of Driver's Right Joystick
     * @return the value [0,1] in double
     */
    public double getDriverRX(){
        return driverController.getRightX();
    }

    /**
     * get the value of Left Trigger
     * @return the value [0,1] in double
     */
    public double getDriverLT(){
        return driverController.getLeftTriggerAxis();
    }

    /**
     * get the value of Right Trigger
     * @return the value [0,1] in double
     */
    public double getDriverRT(){
        return driverController.getRightTriggerAxis();
    }

    /**
     * set the Rumble State
     * @param rumbleState the level of rumble you want, see {@link RUMBLE_STATE} for more detials
     */
    public void setRumble(RUMBLE_STATE rumbleState){
        this.rumbleState = rumbleState;
    }

    /**
     * get the state of Driver Controller's button
     * @param id button's index start at 1
     * @return True if button is <b>pressed</b>
     */
    public boolean getDriverButton(int id){
        return driverController.getRawButton(id);
    }

    @Override
    public void periodic() {
        switch (this.rumbleState) {
            case FULL:
                driverController.setRumble(RumbleType.kBothRumble, 0.8);
                break;
            case HALF:
                driverController.setRumble(RumbleType.kBothRumble, 0.4);
                break;
            case QUARTER:
                driverController.setRumble(RumbleType.kBothRumble, 0.2);
                break;
            default:
                driverController.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    public enum RUMBLE_STATE{
        FULL,
        HALF,
        QUARTER,
        DISABLE
    }
}
