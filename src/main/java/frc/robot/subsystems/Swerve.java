package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RobotConstants;
import monologue.Logged;
import monologue.Annotations.Log;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants;
import monologue.Logged;
import monologue.Annotations.Log;

class SwerveModule implements Logged {

    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private SparkPIDController driveMotorPID;
    private SparkPIDController turnMotorPID;

    private DCMotorSim driveSimMotor;
    private DCMotorSim turnSimMotor;

    private SimpleMotorFeedforward driveSimFF;

    private PIDController driveSimPID;

    private PIDController turnSimPID;

    private boolean isReal;

    @Log
    private double driveVelocity;
    @Log
    private Rotation2d turnPosition;
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
    private Rotation2d turnSetpoint;

    public SwerveModule(boolean isReal, int drivePort, int turnPort) {
        this.isReal = isReal;
        turnSetpoint = new Rotation2d(0);
        if (isReal) {
            driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
            turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);

            driveMotorPID = driveMotor.getPIDController();
            turnMotorPID = turnMotor.getPIDController();

            driveMotorPID.setP(Constants.MotorDefinitions.driveMotor.P);
            driveMotorPID.setI(Constants.MotorDefinitions.driveMotor.I);
            driveMotorPID.setD(Constants.MotorDefinitions.driveMotor.D);
            driveMotorPID.setFF(Constants.MotorDefinitions.driveMotor.FF);

            turnMotorPID.setP(Constants.MotorDefinitions.turnMotor.P);
            turnMotorPID.setI(Constants.MotorDefinitions.turnMotor.I);
            turnMotorPID.setD(Constants.MotorDefinitions.turnMotor.D);

            turnMotorPID.setPositionPIDWrappingEnabled(true);
            turnMotorPID.setPositionPIDWrappingMaxInput(2 * Math.PI);
            turnMotorPID.setPositionPIDWrappingMinInput(0);
            
        } else {
            driveSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
            turnSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);

            driveSimFF = new SimpleMotorFeedforward(0.0, 0.0021);

            driveSimPID = new PIDController(0.08, 0.0, 0.00);
            turnSimPID = new PIDController(0.08, 0.0, 0.00);

            turnSimPID.enableContinuousInput(0, 2 * Math.PI);
        }
    }


    public void periodic() {
        if(isReal){
            driveMotorPID.setReference(driveSetpoint, ControlType.kVelocity);
            turnMotorPID.setReference(turnSetpoint.getRadians(), ControlType.kPosition);
        }
        else{
            driveCurrent = driveSimMotor.getCurrentDrawAmps();
            driveVelocity = driveSimMotor.getAngularVelocityRPM() * PIDConstants.WHEEL_CIRCUMFERENCE.in(Meters) * PIDConstants.DRIVE_GEARING;

            turnPosition = Rotation2d.fromRadians(turnSimMotor.getAngularPositionRad());
            turnCurrent = turnSimMotor.getCurrentDrawAmps();

            driveAppliedVoltage = driveSimPID.calculate(driveVelocity, driveSetpoint)
                    + driveSimFF.calculate(driveSetpoint);

            turnAppliedVoltage = turnSimPID.calculate(turnPosition.getRadians(), turnSetpoint.getRadians());

            driveSimMotor.setInputVoltage(driveAppliedVoltage);
            turnSimMotor.setInputVoltage(turnAppliedVoltage);
        }
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, turnPosition);
        turnSetpoint = state.angle;
        driveSetpoint = state.speedMetersPerSecond * Math.cos(turnSetpoint.getRadians() - turnPosition.getRadians());

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVelocity, turnPosition);
    }
}

public class Swerve extends SubsystemBase implements Logged {

    private SwerveModule[] modules;

    private SimGyro simGyro;

    private final boolean isReal;

    SwerveDriveKinematics kinematics;

    public Swerve(boolean isReal) {
        simGyro = new SimGyro();
        this.isReal = isReal;

        modules = new SwerveModule[4];

        modules[0] = new SwerveModule(isReal, MotorConstants.FRONT_LEFT_DRIVE, MotorConstants.FRONT_LEFT_TURN);
        modules[1] = new SwerveModule(isReal, MotorConstants.FRONT_RIGHT_DRIVE, MotorConstants.FRONT_RIGHT_TURN);
        modules[2] = new SwerveModule(isReal, MotorConstants.BACK_LEFT_DRIVE, MotorConstants.BACK_LEFT_TURN);
        modules[3] = new SwerveModule(isReal, MotorConstants.BACK_RIGHT_DRIVE, MotorConstants.BACK_RIGHT_TURN);

    }


    @Override
    public void periodic() {
        modules[0].periodic();
        modules[1].periodic();
        modules[2].periodic();
        modules[3].periodic();

        if (!isReal) {
            simGyro.update(this.getModuleStates());
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, RobotConstants.MAX_LINEAR_SPEED);

        modules[0].setState(states[0]);
        modules[1].setState(states[1]);
        modules[2].setState(states[2]);
        modules[3].setState(states[3]);
    }
    
    @Log
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            modules[0].getState(),
            modules[1].getState(),
            modules[2].getState(),
            modules[3].getState()
        };
    }

    public Rotation2d getGyroAngle() {
        return isReal ? new Rotation2d() : simGyro.getAngle();
    }
}

class SimGyro implements Logged {
    @Log
    private double angle;

    public SimGyro() {
        angle = 0;
    }

    public void update(SwerveModuleState[] states) {
        ChassisSpeeds speeds = PIDConstants.kinematics.toChassisSpeeds(states);

        angle += speeds.omegaRadiansPerSecond * 0.02;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(angle);
    }
}


