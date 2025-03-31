package frc.robot.Command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Drivetrain;

public class AutoCommand extends Command{
    private Timer timer;
    private boolean isFinished;
    private Drivetrain drive;

    public AutoCommand(Drivetrain drivetrain){
        timer = new Timer();
        isFinished = false;
        this.drive = drivetrain;
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    public void execute(){
        double time = timer.get();

        for(int i = 0; i < 4; i++){
            if(time < 1){
                drive.drive(0.5, 0, false);
            }
            drive.turnByGyro(90, -0.5);
            timer.reset();
        }    
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted){
        drive.drive(0, 0, false);;
        timer.stop();
    }
    
}
