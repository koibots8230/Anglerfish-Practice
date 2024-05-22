package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import frc.lib.util.FFClass;
import frc.lib.util.PIDClass;

public class Constants {
    public static class ShooterConstants {
        public static final Measure<Velocity<Angle>> TOP_AMP_SPEED = RPM.of(0);
        public static final Measure<Velocity<Angle>> BOTTOM_AMP_SPEED = RPM.of(0);
        public static final Measure<Velocity<Angle>> TOP_SPEAKER_SPEED = RPM.of(0);
        public static final Measure<Velocity<Angle>> BOTTOM_SPEAKER_SPEED = RPM.of(0);

        public static final PIDClass TOP_PID = new PIDClass(0, 0, 0, 0, 0, 0);
        public static final PIDClass BOTTOM_PID = new PIDClass(0, 0, 0, 0, 0, 0);

        public static final FFClass TOP_FF = new FFClass(0, 0, 0, 0, 0, 0, 0, 0);
        public static final FFClass BOTTOM_FF = new FFClass(0, 0, 0, 0, 0, 0, 0, 0);

        public static final int TOP_ID = 11;
        public static final int BOTTOM_ID = 13;
    }

    public static class IntakeConstants {
        public static final Measure<Velocity<Angle>> SPEED = RPM.of(0);
        public static final PIDClass PID = new PIDClass(0, 0, 0, 0, 0, 0);
        public static final FFClass FF = new FFClass(0, 0, 0, 0, 0, 0, 0, 0);

        public static final int ID = 14;
    }

    public static class IndexerConstants {
        public static final PIDClass PID = new PIDClass(0, 0, 0, 0, 0, 0);
        public static final FFClass FF = new FFClass(0, 0, 0, 0, 0, 0, 0, 0);

        public static final int ID = 9;
    }

    public static class RobotConstants {
        public static final Measure<Time> PERIODIC_LOOP = Milliseconds.of(20);
    }
}
