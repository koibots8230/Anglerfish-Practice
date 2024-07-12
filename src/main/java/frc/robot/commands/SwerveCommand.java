package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class SwerveCommand extends Command {

    public final Swerve SwerveSubsystem;
    public final DoubleSupplier leftX;
    public final DoubleSupplier leftY;
    public final DoubleSupplier rightX;
    public final DoubleSupplier rightY;
    
    public SwerveCommand(DoubleSupplier leftX, DoubleSupplier leftY,DoubleSupplier rightX,DoubleSupplier rightY, Swerve swerve){
        SwerveSubsystem = swerve;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;
        addRequirements(swerve);

    }

    @Override
    public void execute() {
        ChassisSpeeds desiredSpeed = new ChassisSpeeds(leftX.getAsDouble(), leftY.getAsDouble(), rightX.getAsDouble());
        SwerveSubsystem.setChassisSpeed(desiredSpeed);
    }
}
