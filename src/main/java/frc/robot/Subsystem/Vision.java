package frc.robot.Subsystem;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    private final PhotonCamera camera;
    private PhotonTrackedTarget currentTarget;
    private boolean hasTarget = false;
    private boolean isEnabled = true;

    public Vision(String cameraName){
        camera = new PhotonCamera("photovision");
    }

    public void enable(){
        isEnabled = true;
        camera.setDriverMode(false); //Switch back to vision processing mode
    }

    public void disable(){
        isEnabled = false;
        camera.setDriverMode(true); //Switch to basic driver mode
        hasTarget = false;
        currentTarget = null;
    }

    public boolean isEnabled(){
        return isEnabled;
    }

    @Override
    public void periodic(){
        if(!isEnabled) return;
        
        var result = camera.getLatestResult();
        hasTarget = result.hasTargets();

        if(hasTarget){
            currentTarget = result.getBestTarget();
            SmartDashboard.putNumber("Target Yaw", currentTarget.getYaw());
            SmartDashboard.putNumber("Target Pitch", currentTarget.getPitch());
            SmartDashboard.putNumber("Target Area", currentTarget.getArea());
        }else{
            SmartDashboard.putBoolean("Has Target", false);
        }
    }

    public double getTargetYaw() {
        return hasTarget ? currentTarget.getYaw() : 0;
    }

    public boolean hasTarget(){
        return hasTarget;
    }
}
