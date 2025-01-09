package frc.robot.utils;

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
        L,      // Left
        R,      // Right
        MID     // Mid or Manual Control
    }

    public enum Level{
        L1,
        L2,
        L3
    }

    public enum Mode{
        INTAKE,
        PICK,
        PLACE
    }

    public class AutoAimSetting{
        public Spot spot;
        public Level level;
        public Mode mode;
    }
}

