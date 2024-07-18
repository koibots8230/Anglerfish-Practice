package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    public static final class RobotConstants {

        public static final Measure<Distance> ROBOT_WIDTH = Inches.of(21.375);
        public static final Measure<Distance> ROBOT_LENGTH = Inches.of(21.375);

        public static final Measure<Velocity<Distance>> MAX_LINEAR_SPEED = MetersPerSecond.of(4);

        public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPPEd = RadiansPerSecond.of(2 * Math.PI);
    }

    public static class MotorDefinition {
        public double P;
        public double I;
        public double D;
        public double FF;

        public MotorDefinition(double P, double I, double D, double FF) {
            this.P = P;
            this.I = I;
            this.D = D;
            this.FF = FF;
        }
    }

    public static class MotorDefinitions {

        public static MotorDefinition topShooter = new MotorDefinition(0.00005, 0.0, 0.0, 0.000188);

        public static MotorDefinition bottomShooter = new MotorDefinition(0.00018, 0.0, 0.0, 0.000028);

        public static MotorDefinition driveMotor = new MotorDefinition(0.0, 0.0, 0.0, 0.0);

        public static MotorDefinition turnMotor = new MotorDefinition(0, 0, 0, 0);
    }

    public static class PIDConstants {
        // ----------intake----------//

        public static final double INTAKE_PID_KP = 0.0;
        public static final double INTAKE_PID_KI = 0.0;
        public static final double INTAKE_PID_KD = 0.0;

        public static final double INTAKE_FEEDFORWARD_FF = 0.0;

        public static final double INTAKE_SETPOINT = 0.0;

        // ----------indexer----------//

        public static final double INDEXER_PID_KP = 0.0;
        public static final double INDEXER_PID_KI = 0.0;
        public static final double INDEXER_PID_KD = 0.0;

        public static final double INDEXER_FEEDFORWARD_FF = 0.0;

        public static final double INDEXER_SETPOINT = 400;

        // ----------shooter---------- //

        public static final double TOP_SHOOTER_SPEAKER_SETPOINT = -1000; // should be negitive
        public static final double TOP_SHOOTER_AMP_SETPOINT = -600; // should be negitive
        public static final double TOP_SHOOTER_VELOCITY_RANGE_AMP = 20;

        public static final double BOTTOM_SHOOTER_SPEAKER_SETPOINT = 1000;
        public static final double BOTTOM_SHOOTER_AMP_SETPOINT = -600;
        public static final double BOTTOM_SHOOTER_VELOCITY_RANGE_AMP = 20;

        public static final double SEND_TO_SHOOTER_SETPOINT = 800;

        // ----------Swerve---------- //

        public static final double DRIVE_GEARING = 5.08;
        public static final Measure<Distance> WHEEL_CIRCUMFERENCE = Inches.of(3 * Math.PI);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(RobotConstants.ROBOT_LENGTH.divide(2),
                        RobotConstants.ROBOT_WIDTH.divide(2)),
                new Translation2d(RobotConstants.ROBOT_LENGTH.divide(2),
                        RobotConstants.ROBOT_WIDTH.divide(-2)),
                new Translation2d(RobotConstants.ROBOT_LENGTH.divide(-2),
                        RobotConstants.ROBOT_WIDTH.divide(2)),
                new Translation2d(RobotConstants.ROBOT_LENGTH.divide(-2),
                        RobotConstants.ROBOT_WIDTH.divide(-2)));

    }

    public static class MotorConstants {

        public static final int BACK_LEFT_TURN = 5;
        public static final int BACK_LEFT_DRIVE = 4;
        public static final int FRONT_RIGHT_DRIVE = 7;
        public static final int FRONT_RIGHT_TURN = 8;
        public static final int BACK_RIGHT_DRIVE = 2;
        public static final int BACK_RIGHT_TURN = 3;
        public static final int FRONT_LEFT_DRIVE = 6;
        public static final int FRONT_LEFT_TURN = 1;
        public static final int INDEXER = 9;
        public static final int SHOOTER_TOP = 11;
        public static final int SHOOTER_BOTTOM = 13;
        public static final int INTAKE = 14;

        private static final int DRIVING_PINION_TEETH = 13;
        public static final double DRIVE_GEAR_RATIO = (45.0 * 22) / (DRIVING_PINION_TEETH * 15);
        public static final double TURN_GEAR_RATIO = (62.0 / 14) * 12;

    }

    public static class controller {

        public static final XboxController CONTROLLER = new XboxController(0);

        public static final GenericHID OPERATOR_CONTROLLER = new GenericHID(1);

    }

    public static class LoggerConstants {
        public static final boolean FILEONLY = false;
        public static final boolean LAZYLOGGING = false;
    }

}
