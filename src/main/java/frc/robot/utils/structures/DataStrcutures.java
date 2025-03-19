package frc.robot.utils.structures;

import java.util.HashMap;

public class DataStrcutures{
    public enum Station{
        A,
        B,
        C,
        D,
        E,
        F
    }
    
    public enum Spot{
        L(0),      // Left
        R(1),      // Right
        MID(2);     // Mid or Manual Control

        public final int value;

        Spot(int initValue)
        {
            this.value = initValue;
        }

        private static HashMap<Integer, Spot> _map = null;
        static
        {
            _map = new HashMap<Integer, Spot>();
            for (Spot type : Spot.values())
            {
                _map.put(type.value, type);
            }
        }

        /**
         * Gets Spot from specified value
         * @param value Value of Spot
         * @return Spot of specified value
         */
        public static Spot valueOf(int value)
        {
            Spot retval = _map.get(value);
            if (retval != null) return retval;
            return Spot.values()[0];
        }
    }

    public enum Level{
        L1(0),
        L2(1),
        L3(2),
        L4(3),
        LClimb(4);

        public final int value;

        Level(int initValue)
        {
            this.value = initValue;
        }

        private static HashMap<Integer, Level> _map = null;
        static
        {
            _map = new HashMap<Integer, Level>();
            for (Level type : Level.values())
            {
                _map.put(type.value, type);
            }
        }

        /**
         * Gets Level from specified value
         * @param value Value of Level
         * @return Level of specified value
         */
        public static Level valueOf(int value)
        {
            Level retval = _map.get(value);
            if (retval != null) return retval;
            return Level.values()[0];
        }
    }

    public enum Mode{
        CORAL_INTAKE(0),
        CORAL_PLACE(1),
        ALGAE_INTAKE(2);

        public final int value;

        Mode(int initValue)
        {
            this.value = initValue;
        }

        private static HashMap<Integer, Mode> _map = null;
        static
        {
            _map = new HashMap<Integer, Mode>();
            for (Mode type : Mode.values())
            {
                _map.put(type.value, type);
            }
        }

        /**
         * Gets Mode from specified value
         * @param value Value of Mode
         * @return Mode of specified value
         */
        public static Mode valueOf(int value)
        {
            Mode retval = _map.get(value);
            if (retval != null) return retval;
            return Mode.values()[0];
        }
    }
}
