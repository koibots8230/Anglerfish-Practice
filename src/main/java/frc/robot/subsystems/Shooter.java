package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Robot;
import monologue.Logged;
import monologue.Annotations.Log;

public class Shooter extends SubsystemBase implements Logged {

    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;

    private DCMotorSim topSimMotor;
    private DCMotorSim bottomSimMotor;
    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder;
    private SparkPIDController topShoterPID;
    private SparkPIDController bottomShoterPID;

    private SimpleMotorFeedforward topSimFF;

    private SimpleMotorFeedforward bottomSimFF;

    private PIDController topSimPID;

    private PIDController bottomSimPID;

    private final boolean isReal;

    @Log
    private double topShoterVelocity;
    @Log
    private double bottomShoterVelocity;

    @Log
    private double  bottomShoterCurrent;
    @Log
    private double  topshoterCurrent;
    @Log
    private double  topAppliedVoltage;
    @Log
    private double bottomAppliedVoltage;
    @Log
    private double topShooterSetpoint;
    @Log
    private double bottomShooterSetpoint;



    public Shooter(boolean isReal) {
        this.isReal = isReal;
        if (isReal) {
            topMotor = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);
            bottomMotor = new CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless);


            topShoterPID = topMotor.getPIDController();

            bottomShoterPID = bottomMotor.getPIDController();

            topEncoder = topMotor.getEncoder();

            bottomEncoder = bottomMotor.getEncoder();

            topShoterPID.setP(Constants.MotorDefinitions.topShooter.P);
            topShoterPID.setI(Constants.MotorDefinitions.topShooter.I);
            topShoterPID.setD(Constants.MotorDefinitions.topShooter.D);
            topShoterPID.setFF(Constants.MotorDefinitions.topShooter.FF);

            bottomShoterPID.setP(Constants.MotorDefinitions.bottomShooter.P);
            bottomShoterPID.setP(Constants.MotorDefinitions.bottomShooter.I);
            bottomShoterPID.setP(Constants.MotorDefinitions.bottomShooter.D);
            bottomShoterPID.setP(Constants.MotorDefinitions.bottomShooter.FF);
        }

        else {
            topSimMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
            bottomSimMotor = new DCMotorSim(DCMotor.getNEO(1),1,1);

            topSimFF = new SimpleMotorFeedforward(0.0,0.0021);
            bottomSimFF = new SimpleMotorFeedforward(0.0,0.0021);

            topSimPID = new PIDController(0.08,0.0,0.00);
            bottomSimPID = new PIDController(0.08, 0.0, 0.00);

        }


    }

    @Override
    public void periodic(){
        if(isReal){
            topShoterPID.setReference(topShooterSetpoint, CANSparkBase.ControlType.kVelocity);
            bottomShoterPID.setReference(bottomShooterSetpoint, CANSparkBase.ControlType.kVelocity);

        }
    }

    @Override
    public void simulationPeriodic(){
        topSimMotor.update(.02);
        bottomSimMotor.update(.02);

        topshoterCurrent = topSimMotor.getCurrentDrawAmps();
        topShoterVelocity = topSimMotor.getAngularVelocityRPM();

        bottomShoterVelocity = bottomSimMotor.getAngularVelocityRPM();
        bottomShoterCurrent = bottomSimMotor.getCurrentDrawAmps();

        topAppliedVoltage =
                topSimPID.calculate(topShoterVelocity, topShooterSetpoint) + topSimFF.calculate(topShooterSetpoint);
        bottomAppliedVoltage = bottomSimPID.calculate(bottomShoterVelocity, bottomShooterSetpoint) + bottomSimFF.calculate(bottomShooterSetpoint);

        topSimMotor.setInputVoltage(topAppliedVoltage);
        bottomSimMotor.setInputVoltage(bottomAppliedVoltage);
    }

    public void setVelocity(double topShooterSetpoint, double bottomShooterSetpoint) {
        this.topShooterSetpoint = topShooterSetpoint;
        this.bottomShooterSetpoint = bottomShooterSetpoint;
    }

    public boolean checkVelocity() {
        return Math.abs(topEncoder.getVelocity() - PIDConstants.TOP_SHOOTER_AMP_SETPOINT) <= PIDConstants.TOP_SHOOTER_VELOCITY_RANGE_AMP
                &&
                Math.abs(bottomEncoder.getVelocity() - PIDConstants.BOTTOM_SHOOTER_AMP_SETPOINT) <= PIDConstants.BOTTOM_SHOOTER_VELOCITY_RANGE_AMP
                ||
                Math.abs(topEncoder.getVelocity() - PIDConstants.TOP_SHOOTER_SPEAKER_SETPOINT) <= PIDConstants.TOP_SHOOTER_VELOCITY_RANGE_AMP
                        &&
                        Math.abs(bottomEncoder.getVelocity() - PIDConstants.BOTTOM_SHOOTER_SPEAKER_SETPOINT) <= PIDConstants.BOTTOM_SHOOTER_VELOCITY_RANGE_AMP;
    }


}
