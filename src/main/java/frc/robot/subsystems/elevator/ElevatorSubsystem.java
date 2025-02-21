package frc.robot.subsystems.elevator;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance;
    private static ELEVATOR_STATES states;

    private ElevatorIO elevatorIO = new ElevatorKraken();
    
    public ElevatorSubsystem(){
        states = ELEVATOR_STATES.LEVEL_1;
    }

    public ElevatorSubsystem getInstance(){
        if(instance == null) instance = new ElevatorSubsystem();
        return instance;
    }

    @Override
    public void periodic() {
        switch(states){
            case LEVEL_1:
                elevatorIO.setPosition(0.0);
                break;
            case LEVEL_2:
                elevatorIO.setPosition(1.0);
                break;
            case LEVEL_3:
                elevatorIO.setPosition(2.0);
                break;
            case LEVEL_4:
                elevatorIO.setPosition(3.0);
                break;
            default:
                // Handle unexpected states
                break;
        }
    }

    public enum ELEVATOR_STATES {
        LEVEL_1(0),
        LEVEL_2(1),
        LEVEL_3(2),
        LEVEL_4(3);

        public final int value;

        ELEVATOR_STATES(int initValue)
        {
            this.value = initValue;
        }

        private static HashMap<Integer, ELEVATOR_STATES> _map = null;
        static
        {
            _map = new HashMap<Integer, ELEVATOR_STATES>();
            for (ELEVATOR_STATES type : ELEVATOR_STATES.values())
            {
                _map.put(type.value, type);
            }
        }
 
        /**
         * Gets {@link ELEVATOR_STATES} from specified value
         * @param value Value of Spot
         * @return Spot of specified value
         */
        public static ELEVATOR_STATES valueOf(int value)
        {
            ELEVATOR_STATES retval = _map.get(value);
            if (retval != null) return retval;
            return ELEVATOR_STATES.values()[0];
        }
    }
}