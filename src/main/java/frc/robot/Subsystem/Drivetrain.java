package frc.robot.Subsystem;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase {
    private static final int LeftMotorChannel = 0;
    private static final int RightMotorChannel = 1;

    private VictorSP rightmotor;
    private VictorSP leftmotor;

    private DifferentialDrive drive;
    private ADXRS450_Gyro gyro;

    private Rotation2d gyroOffset;
    private boolean fieldOriented;

    private final SlewRateLimiter speedLimiter;
    private final SlewRateLimiter turnLimiter;

    private final DifferentialDriveOdometry odometry;
    private final Field2d field;

    public Drivetrain(){
        rightmotor = new VictorSP(RightMotorChannel);
        leftmotor = new VictorSP(LeftMotorChannel);

        rightmotor.setInverted(true);
        leftmotor.setInverted(false);

        drive = new DifferentialDrive(leftmotor, rightmotor);
        gyro = new ADXRS450_Gyro();

        drive.setSafetyEnabled(true);
        drive.setExpiration(0.1); 


        field = new Field2d();
        gyroOffset = new Rotation2d();
        fieldOriented = false;

        odometry = new DifferentialDriveOdometry(getGyroRotation2d(), 0, 0);


        speedLimiter = new SlewRateLimiter(3.0);
        turnLimiter = new SlewRateLimiter(3.0);

        gyro.calibrate();
        gyro.reset();

    }

    public void drive(double speed, double turn, boolean squareInput){
        speed = applyDeadband(speed, 0.02);
        turn = applyDeadband(turn, 0.02);

        speed = speedLimiter.calculate(speed);
        turn = turnLimiter.calculate(turn);

        drive.arcadeDrive(speed, turn, squareInput);
    }

    public void fieldOrientedDrive(double xSpeed, double ySpeed, boolean squareInput){
        if(!fieldOriented){
            drive(xSpeed, ySpeed, squareInput);
            return;
        }

        Rotation2d gyroAngle = getGyroRotation2d();

        var robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 0, gyroAngle);

        double speed = robotSpeeds.vxMetersPerSecond;
        double turn = robotSpeeds.vyMetersPerSecond;

        drive(speed, turn, squareInput);
    }

    public void toggleFieldOriented(){
        fieldOriented = !fieldOriented;

        if(fieldOriented){
            gyroOffset = getGyroRotation2d();
        }
    }

    public void resetFieldOrientation(){
        gyroOffset = getGyroRotation2d();
    }

    public boolean isFieldOriented() {
        return fieldOriented;
    }

    private double applyDeadband(double value, double deadband){
        if(Math.abs(value) < deadband){
            return 0.0;
        }
        return value;
    }

    public void resetGyro(){
        gyro.reset();
        gyroOffset = getGyroRotation2d();
    }

    public double getGyroAngle(){
        return gyro.getAngle();
    }

    public Rotation2d getGyroRotation2d(){
        return Rotation2d.fromDegrees(-gyro.getAngle()).minus(gyroOffset);
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        gyro.reset();
        odometry.resetPosition(getGyroRotation2d(), 0, 0, pose);
    }

    public void stop(){
        speedLimiter.reset(0);
        turnLimiter.reset(0);
        drive.arcadeDrive(0, 0);
    }


    @Override
    public void periodic(){
        odometry.update(getGyroRotation2d(), 0, 0);
        
        field.setRobotPose(getPose());

        SmartDashboard.putNumber("Gyro Angle: ", getGyroAngle());
        SmartDashboard.putData("Field", field);
        SmartDashboard.putBoolean("Field Oriented", fieldOriented);
        SmartDashboard.putString("Robot Pose", getPose().toString());
    }
}
