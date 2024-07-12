package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.function.DoubleSupplier;

import javax.print.URIException;
import javax.swing.text.StyledEditorKit.BoldAction;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RobotConstants;
import monologue.Logged;
import monologue.Annotations.Log;
import com.revrobotics.REVPhysicsSim;

class SwerveModule {

    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private SwerveModuleState swerveModuleState;

    private SwerveModuleState simSwerveModuleState;

    private SparkPIDController driveMotorPID;
    private SparkPIDController turnMotorPID;

    private DCMotorSim driveSimMotor;
    private DCMotorSim turnSimMotor;

    private SimpleMotorFeedforward driveSimFF;

    private SimpleMotorFeedforward turnSimFF;

    private PIDController driveSimPID;

    private PIDController turnSimPID;

    private boolean isReal;

    @Log
    private double driveVelocity;
    @Log
    private double turnVelocity;
    @Log
    private double driveCurrent;
    @Log
    private double turnCurrent;
    @Log
    private double driveAppliedVoltage;
    @Log
    private double turnAppliedVoltage;
    @Log
    private double driveSetpoint;
    @Log
    private double turnSetpoint;

    public SwerveModule(Boolean isreal, int drivePort, int turnPort) {
        this.isReal = isReal;
        if (isreal) {
            driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
            turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
            swerveModuleState = new SwerveModuleState();

            driveMotorPID = driveMotor.getPIDController();
            turnMotorPID = turnMotor.getPIDController();

            driveMotorPID.setP(Constants.MotorDefinitions.driveMotor.P);
            driveMotorPID.setI(Constants.MotorDefinitions.driveMotor.I);
            driveMotorPID.setD(Constants.MotorDefinitions.driveMotor.D);
            driveMotorPID.setFF(Constants.MotorDefinitions.driveMotor.FF);

            turnMotorPID.setP(Constants.MotorDefinitions.turnMotor.P);
            turnMotorPID.setI(Constants.MotorDefinitions.turnMotor.I);
            turnMotorPID.setD(Constants.MotorDefinitions.turnMotor.D);
            turnMotorPID.setFF(Constants.MotorDefinitions.turnMotor.FF);
        } else {
            driveSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
            turnSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);

            simSwerveModuleState = new SwerveModuleState();

            driveSimFF = new SimpleMotorFeedforward(0.0, 0.0021);
            turnSimFF = new SimpleMotorFeedforward(0.0, 0.0021);

            driveSimPID = new PIDController(0.08, 0.0, 0.00);
            turnSimPID = new PIDController(0.08, 0.0, 0.00);

        }
    }

    public void setState(SwerveModuleState state) {
        if(isReal){

       
        swerveModuleState = state;

        driveMotorPID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        turnMotorPID.setReference(state.angle.getDegrees() / 360, ControlType.kPosition);
 }
        else{
        swerveModuleState = state;

        driveCurrent = driveSimMotor.getCurrentDrawAmps();
        driveVelocity = driveSimMotor.getAngularVelocityRPM();

        turnVelocity = turnSimMotor.getAngularVelocityRPM();
        turnCurrent = turnSimMotor.getCurrentDrawAmps();

        driveAppliedVoltage = driveMotorPID.calculate(driveVelocity, state.speedMetersPerSecond)
                + driveSimFF.calculate(state.speedMetersPerSecond);
        turnAppliedVoltage = turnMotorPID.calculate(turnVelocity, state.speedMetersPerSecond)
                + turnSimFF.calculate(state.speedMetersPerSecond);

        driveSimMotor.setInputVoltage(driveAppliedVoltage);
        turnSimMotor.setInputVoltage(turnAppliedVoltage);
        
        }
    }

    public SwerveModuleState SwerveState() {
        return swerveModuleState;

    }
}

public class Swerve extends SubsystemBase implements Logged {

    private SwerveModule FLModule;
    private SwerveModule FRModule;
    private SwerveModule BLModule;
    private SwerveModule BRModule;

    private Translation2d FLMotorLocation;
    private Translation2d FRMotorLocation;
    private Translation2d BLMotorLocation;
    private Translation2d BRMotorLocation;

    SwerveDriveKinematics kinematics;

    public Swerve(boolean isReal) {

        FLModule = new SwerveModule(isReal, MotorConstants.FRONT_LEFT_DRIVE, MotorConstants.FRONT_LEFT_TURN);
        FRModule = new SwerveModule(isReal, MotorConstants.FRONT_RIGHT_DRIVE, MotorConstants.FRONT_RIGHT_TURN);
        BLModule = new SwerveModule(isReal, MotorConstants.BACK_LEFT_DRIVE, MotorConstants.BACK_LEFT_TURN);
        BRModule = new SwerveModule(isReal, MotorConstants.BACK_RIGHT_DRIVE, MotorConstants.BACK_RIGHT_TURN);
        FLMotorLocation = new Translation2d(RobotConstants.ROBOT_LENGTH.divide(2),
                RobotConstants.ROBOT_WIDTH.divide(2));
        FRMotorLocation = new Translation2d(RobotConstants.ROBOT_LENGTH.divide(2),
                RobotConstants.ROBOT_WIDTH.divide(-2));
        BLMotorLocation = new Translation2d(RobotConstants.ROBOT_LENGTH.divide(-2),
                RobotConstants.ROBOT_WIDTH.divide(2));
        BRMotorLocation = new Translation2d(RobotConstants.ROBOT_LENGTH.divide(-2),
                RobotConstants.ROBOT_WIDTH.divide(-2));

        kinematics = new SwerveDriveKinematics(
                FLMotorLocation,
                FRMotorLocation,
                BLMotorLocation,
                BRMotorLocation);

    }

    public void setChassisSpeed(ChassisSpeeds Velocity) {
        SwerveModuleState[] stateVelocity = kinematics.toSwerveModuleStates(Velocity);

        FLModule.setState(stateVelocity[0]);
        FRModule.setState(stateVelocity[1]);
        BLModule.setState(stateVelocity[2]);
        BRModule.setState(stateVelocity[3]);
    }

}
