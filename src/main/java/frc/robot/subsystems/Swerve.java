package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

class SwerveModule{

    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;
    private SwerveModuleState swerveModuleState;

    public SwerveModule(int divePort, int turnPort){
      driveMotor = new CANSparkMax(0, MotorType.kBrushless);
      turnMotor = new CANSparkMax(1, MotorType.kBrushless);
      swerveModuleState = new SwerveModuleState();

    }

    public void setState(SwerveModuleState stateVelocity){
        swerveModuleState = stateVelocity;
    }

    public SwerveModuleState SwerveState(){
        return swerveModuleState;

    }
}

public class Swerve extends SubsystemBase {

    SwerveModule FLModule = new SwerveModule(0,1);
    SwerveModule FRModule = new SwerveModule(2, 3);
    SwerveModule BLModule = new SwerveModule(4, 5);
    SwerveModule BRModule = new SwerveModule(6, 7);

    Translation2d FLMotorLocation = new Translation2d(RobotConstants.ROBOT_LENGTH.divide(2), RobotConstants.ROBOT_WIDTH.divide(2));
    Translation2d FRMotorLocation = new Translation2d(RobotConstants.ROBOT_LENGTH.divide(2), RobotConstants.ROBOT_WIDTH.divide(-2));
    Translation2d BLMotorLocation = new Translation2d(RobotConstants.ROBOT_LENGTH.divide(-2), RobotConstants.ROBOT_WIDTH.divide(2));
    Translation2d BRMotorLocation = new Translation2d(RobotConstants.ROBOT_LENGTH.divide(-2), RobotConstants.ROBOT_WIDTH.divide(-2));

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    FLMotorLocation,
    FRMotorLocation,
    BLMotorLocation,
    BRMotorLocation
    );

    public Swerve(boolean isReal) {

    }

    

    public void setChassisSpeed(ChassisSpeeds Velocity){
        SwerveModuleState[] stateVelocity = kinematics.toSwerveModuleStates(Velocity);

        FLModule.setState(stateVelocity[0]);
        FRModule.setState(stateVelocity[1]);
        BLModule.setState(stateVelocity[2]);
        BRModule.setState(stateVelocity[3]);
    }

    
    public void periodic(double leftX, double leftY, double rightX, double rightY) {

        ChassisSpeeds desiredSpeed = new ChassisSpeeds(leftX, leftY, rightX);

        setChassisSpeed(desiredSpeed);

        double logstate[] = {
            FLModule.SwerveState().angle.getDegrees(),
            FLModule.SwerveState().speedMetersPerSecond,
            FRModule.SwerveState().angle.getDegrees(),
            FRModule.SwerveState().speedMetersPerSecond,
            BLModule.SwerveState().angle.getDegrees(),
            BLModule.SwerveState().speedMetersPerSecond,
            BRModule.SwerveState().angle.getDegrees(),
            BRModule.SwerveState().speedMetersPerSecond,
        };

        SmartDashboard.putNumberArray("SwerveModuleState", logstate);
    }

}
