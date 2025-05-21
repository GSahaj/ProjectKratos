package frc.robot.Subsystem;

//WPIlib FRC imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase{
    public Drivetrain(){}

    //Applies the deadband and the Limiters, Using the squareInput --> Changes the Input of the Joystick
    public void drive(double speed, double turn, boolean squareInput){
        speed = applyDeadband(speed, 0.02);
        turn = applyDeadband(turn, 0.02);

        speed = RobotMap.variables.speedLimiter.calculate(speed);
        turn = RobotMap.variables.turnLimiter.calculate(turn);

        RobotMap.variables.drive.arcadeDrive(speed, turn, squareInput);
    }

    public void curvatureDrive(double speed, double turn, boolean squareInput){
        speed = applyDeadband(speed, 0.02);
        turn = applyDeadband(turn, 0.02);

        speed = RobotMap.variables.speedLimiter.calculate(speed);
        turn = RobotMap.variables.turnLimiter.calculate(turn);

        RobotMap.variables.drive.curvatureDrive(speed, turn, squareInput);
    }

    //Method to Remove Drift, A deadband is applied to remove the anomalies away from the netural zone
    private double applyDeadband(double value, double deadband){
        if(Math.abs(value) < deadband){
            return 0.0;
        }
        return value;
    }    

    //Makes the robot field-orientated and face the drive
    public void fieldOrientedDrive(double xSpeed, double ySpeed, boolean squareInput){
        if(!RobotMap.variables.fieldOriented){
            drive(xSpeed, ySpeed, squareInput);
            return;
        }

        Rotation2d gyroAngle = getGyroRotation2d();

        var robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 0, gyroAngle); //Uses Calculations to calulate relative field orientation

        double speed = robotSpeeds.vxMetersPerSecond;
        double turn = robotSpeeds.vyMetersPerSecond;

        drive(speed, turn, squareInput);
    }

    public boolean isFieldOriented() {
        return RobotMap.variables.fieldOriented;
    }

    //Toggle between robot-drive and field-oriented drive
    public void toggleFieldOriented(){
        RobotMap.variables.fieldOriented = !RobotMap.variables.fieldOriented;

        if(RobotMap.variables.fieldOriented){
            RobotMap.variables.gyroOffset = getGyroRotation2d();
        }
    }

    //reset fied orientated drive
    public void resetFieldOrientation(){
        RobotMap.variables.gyroOffset = getGyroRotation2d();
    }

    public void resetGyro(){
        RobotMap.variables.gyro.reset();
        RobotMap.variables.gyroOffset = getGyroRotation2d();
    }

    //Get gyro Angle
    public double getGyroAngle(){
        return RobotMap.variables.gyro.getAngle();
    }

    public void turnByGyro(double gyroAngle, double turnSpeed){
        RobotMap.variables.gyro.reset();
        while(Math.abs(getGyroAngle()) < gyroAngle){
            drive(0, turnSpeed, false);
        }
        drive(0, 0, false);
    }

    //Converts Gyro Angle to Rotation2D
    public Rotation2d getGyroRotation2d(){
        return Rotation2d.fromDegrees(-RobotMap.variables.gyro.getAngle()).minus(RobotMap.variables.gyroOffset);
    }

    public Pose2d getPose(){
        return RobotMap.variables.odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        RobotMap.variables.gyro.reset();
        RobotMap.variables.odometry.resetPosition(getGyroRotation2d(), 0, 0, pose);
    }

    public void stop(){
        RobotMap.variables.speedLimiter.reset(0);
        RobotMap.variables.turnLimiter.reset(0);
        RobotMap.variables.drive.arcadeDrive(0, 0);
    }


    @Override
    public void periodic(){
        RobotMap.variables.odometry.update(getGyroRotation2d(), 0, 0);
        
        RobotMap.variables.field.setRobotPose(getPose());

        SmartDashboard.putNumber("Gyro Angle: ", getGyroAngle());
        SmartDashboard.putData("Field", RobotMap.variables.field);
        SmartDashboard.putBoolean("Field Oriented", RobotMap.variables.fieldOriented);
        SmartDashboard.putString("Robot Pose", getPose().toString());
    }
}
