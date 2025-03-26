package frc.robot.Command;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Drivetrain;

public class Driver extends Command{
    private final Drivetrain drivetrain;
    private final Joystick controller;

    private final int fieldOrientedButton;

    private final int resetOrientationButton;

    private boolean squareInput = true;

    public Driver(Drivetrain drivetrain, Joystick controller, int fieldOrientedButton, int resetOrientationButton){
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.fieldOrientedButton = fieldOrientedButton;
        this.resetOrientationButton = resetOrientationButton;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.resetGyro();
    }

    @Override
    public void execute(){
        double speed = -controller.getY();
        double turn = controller.getX();

        if(controller.getRawButton(fieldOrientedButton)){
            drivetrain.toggleFieldOriented();
        }

        if(controller.getRawButton(resetOrientationButton)){
            drivetrain.resetFieldOrientation();
        }

        if(controller.getRawButton(3)){
            squareInput = !squareInput;
        }

        if(drivetrain.isFieldOriented()){
            drivetrain.fieldOrientedDrive(speed, turn, squareInput);
        }else{
            drivetrain.drive(speed, turn, squareInput);
        }

    }

    @Override
    public void end(boolean interrupted){
        drivetrain.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
