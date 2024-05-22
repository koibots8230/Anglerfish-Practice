package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Indexer extends SubsystemBase implements Logged {

    
    private DigitalInput indexerDigitalInput;

    private CANSparkMax indexerMotor;
    
    private SparkPIDController indexerPID;

    private boolean isReal;
    @Log
    private double indexerSetpoint;

    private DCMotorSim simMotor;

    private PIDController simPID;

    private SimpleMotorFeedforward simFF;
    @Log
    private double MotorCurrent;
    @Log
    private double MotorAppliedVoltage;
    @Log
    private double MotorVelocity;

    public Indexer(boolean isReal) {
        this.isReal = isReal;
        if(this.isReal){
        indexerDigitalInput = new DigitalInput(0);
        indexerMotor = new CANSparkMax(2, MotorType.kBrushless);
        indexerPID = indexerMotor.getPIDController();

        indexerPID.setP(PIDConstants.INDEXER_PID_KP);
        indexerPID.setI(PIDConstants.INDEXER_PID_KI);
        indexerPID.setD(PIDConstants.INDEXER_PID_KD);

        indexerPID.setFF(PIDConstants.INDEXER_FEEDFORWARD_FF);
        }
        else{
            simMotor = new DCMotorSim(DCMotor.getNEO(1),1,1);
            simPID = new PIDController(0.00021, 0.00, 0);
            simFF = new SimpleMotorFeedforward(0.00021, 0.0);
        }
        
    }

    @Override
    public void periodic(){
        if(this.isReal){
            indexerPID.setReference(indexerSetpoint, ControlType.kVelocity);
            MotorCurrent = indexerMotor.getOutputCurrent();
            MotorVelocity = indexerMotor.getEncoder().getVelocity();
            MotorAppliedVoltage = indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage();
        }
    }


    @Override
    public void simulationPeriodic(){
        simMotor.update(.02);
        MotorCurrent = simMotor.getCurrentDrawAmps();
        MotorVelocity = simMotor.getAngularVelocityRPM();
        MotorAppliedVoltage = simPID.calculate(MotorVelocity, indexerSetpoint) + simFF.calculate(indexerSetpoint);
        simMotor.setInputVoltage(MotorAppliedVoltage);
    }


    public void setIndexerVelocity(double IndexerVelocity) {
        indexerSetpoint = IndexerVelocity;
    }

    @Log
    public boolean indexerNoteDetected() {
        return indexerDigitalInput.get();
    }

}
