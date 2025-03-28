package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Drivetrain;
import frc.robot.Subsystem.Vision;

public class VisionCommand extends Command{
    private final Drivetrain drivetrain;
    private final Vision vision;
    private final double toleranceDegrees;
    private final double turnSpeed;

    public VisionCommand(Drivetrain drivetrain, Vision vision, double toleranceDegrees, double turnSpeed){
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.toleranceDegrees = toleranceDegrees;
        this.turnSpeed = turnSpeed;

        addRequirements(drivetrain);
    }
    
    @Override
    public void execute(){
        if(vision.hasTarget()){
            double yaw = vision.getTargetYaw();
            double turn = Math.copySign(turnSpeed, yaw);
            drivetrain.drive(0, turn, false);
        }
    }

    @Override
    public boolean isFinished(){
        return vision.hasTarget() && Math.abs(vision.getTargetYaw()) < toleranceDegrees;
    }

    @Override
    public void end(boolean  interrupted){
        drivetrain.stop();
    }
}
