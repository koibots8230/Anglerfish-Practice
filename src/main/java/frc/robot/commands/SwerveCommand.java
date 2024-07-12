package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class SwerveCommand extends Command {

    public final Swerve SwerveSubsystem;
    public final Double leftX;
    public final Double leftY;
    public final Double rightX;
    public final Double rightY;
    
    public SwerveCommand(double leftX,double leftY,double rightX,double rightY, Swerve swerve){
        SwerveSubsystem = swerve;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;
    }

    @Override
    public void execute() {
        SwerveSubsystem.periodic(leftX, leftY, rightX, rightY);
    }
}
