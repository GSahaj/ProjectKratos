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
        //Set the speed and turn parameter to joystick inputs
        double speed = -controller.getY();
        double turn = controller.getX();

        //Toggle between Field Orientated
        if(controller.getRawButton(fieldOrientedButton)){
            drivetrain.toggleFieldOriented();
        }
        
        //Reset Field Orientation
        if(controller.getRawButton(resetOrientationButton)){
            drivetrain.resetFieldOrientation();
        }

        //Disable SquareInput
        if(controller.getRawButton(2)){
            squareInput = !squareInput;
        }

        //Check if Field Orientated
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
