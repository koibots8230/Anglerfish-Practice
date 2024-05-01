package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;

public class Constants {
    public static class ShooterConstants {
        public static final Measure<Velocity<Angle>> TOP_AMP_SPEED = RPM.of(0);
        public static final Measure<Velocity<Angle>> BOTTOM_AMP_SPEED = RPM.of(0);
        public static final Measure<Velocity<Angle>> TOP_SPEAKER_SPEED = RPM.of(0);
        public static final Measure<Velocity<Angle>> BOTTOM_SPEAKER_SPEED = RPM.of(0);

        public static class TopShooterPID {
            public static final double REAL_KP = 0;
            public static final double REAL_KI = 0;
            public static final double REAL_KD = 0;
            public static final double SIM_KP = 0;
            public static final double SIM_KI = 0;
            public static final double SIM_KD = 0;
        }

        public static class BottomShooterPID {
            public static final double REAL_KP = 0;
            public static final double REAL_KI = 0;
            public static final double REAL_KD = 0;
            public static final double SIM_KP = 0;
            public static final double SIM_KI = 0;
            public static final double SIM_KD = 0;
        }

        public static class TopShooterFF {
            public static final double REAL_KS = 0;
            public static final double REAL_KV = 0;
            public static final double REAL_KA = 0;
            public static final double SIM_KS = 0;
            public static final double SIM_KV = 0;
            public static final double SIM_KA = 0;
        }

        public static class BottomShooterFF {
            public static final double REAL_KS = 0;
            public static final double REAL_KV = 0;
            public static final double REAL_KA = 0;
            public static final double SIM_KS = 0;
            public static final double SIM_KV = 0;
            public static final double SIM_KA = 0;
        }

        public static final int TOP_ID = 11;
        public static final int BOTTOM_ID = 13;
    }

    public static class RobotConstants {
        public static final Measure<Time> PERIODIC_LOOP = Milliseconds.of(20);
    }
}
