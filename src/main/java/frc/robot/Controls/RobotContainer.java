package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Command.Driver;
import frc.robot.Command.VisionCommand;
import frc.robot.Subsystem.Drivetrain;
import frc.robot.Subsystem.Vision;

public class RobotContainer {
    private static final int JOYSTICK_PORT = 0;
    private static final int AIM_BUTTON = 1; //example for aim button
    private static final int TOGGLE_VISION_BUTTON = 3; //Button for toggling vision

    private final Joystick controller = new Joystick(JOYSTICK_PORT);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Vision vision = new Vision("photovision");

    private final Driver driver;
    private final VisionCommand visionCommand;
    private boolean visionEnabled = true;

    public RobotContainer(){

        this.driver = new Driver(drivetrain, controller, 5, 6);
        this.visionCommand = new VisionCommand(drivetrain, vision, 1.5, 0.3);

        setDefaultCommand();
        configureButtonBinding();
    }

    private void configureButtonBinding(){
        new JoystickButton(controller, AIM_BUTTON).whileTrue(visionCommand);

        // Fallback implementation using command-based polling ---- Absolutely trash 
        Command toggleVisionCommand = Commands.runOnce(() -> {
            visionEnabled = !visionEnabled;
            if (visionEnabled) vision.enable();
            else vision.disable();
        }).ignoringDisable(true);

        //Toggle between vision
        new JoystickButton(controller, TOGGLE_VISION_BUTTON)
            .onTrue(toggleVisionCommand);  // or .whenPressed(toggleVisionCommand)
                

    }

    private void setDefaultCommand(){
        drivetrain.setDefaultCommand(driver);
    }

    public void disable(){
        drivetrain.stop();
        vision.disable();
    }

}
