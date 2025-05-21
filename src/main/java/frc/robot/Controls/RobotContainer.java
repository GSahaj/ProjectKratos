package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Command.Driver;
import frc.robot.Command.VisionCommand;
import frc.robot.RobotMap;
import frc.robot.Subsystem.Drivetrain;
import frc.robot.Subsystem.Vision;

public class RobotContainer {
private final Joystick controller = new Joystick(RobotMap.OperatorConstants.JOYSTICK_PORT);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Vision vision = new Vision("photovision");

    private final Driver driver;
    private final VisionCommand visionCommand;

    public RobotContainer(){

        this.driver = new Driver(drivetrain, controller, RobotMap.OperatorConstants.FIELD_ORIENTED_BUTTON, RobotMap.OperatorConstants.RESET_FIELD_BUTTON);
        this.visionCommand = new VisionCommand(drivetrain, vision, 1.5, 0.3);

        setDefaultCommand();
        configureButtonBinding();
    }

    private void configureButtonBinding(){
        new JoystickButton(controller, RobotMap.OperatorConstants.AIM_BUTTON).whileTrue(visionCommand);

        // Fallback implementation using command-based polling ---- Absolutely trash 
        Command toggleVisionCommand = Commands.runOnce(() -> {
            RobotMap.variables.visionEnabled = !RobotMap.variables.visionEnabled;
            if (RobotMap.variables.visionEnabled) vision.enable();
            else vision.disable();
        }).ignoringDisable(true);

        //Toggle between vision
        new JoystickButton(controller, RobotMap.OperatorConstants.TOGGLE_VISION_BUTTON).onTrue(toggleVisionCommand);  // or .whenPressed(toggleVisionCommand)
    }

    private void setDefaultCommand(){
        drivetrain.setDefaultCommand(driver);
    }
    public void disable(){
        drivetrain.stop();
        vision.disable();
    }

}
