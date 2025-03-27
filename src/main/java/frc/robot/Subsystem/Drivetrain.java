package frc.robot.Subsystem;

//WPIlib FRF imports
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
    //Hardware for the Motor Channel
    private static final int LeftMotorChannel = 0;
    private static final int RightMotorChannel = 1;

    //Variables for the Motor Controller
    private VictorSP rightmotor;
    private VictorSP leftmotor;

    //Variales for the Differential drive and gyro
    private DifferentialDrive drive;
    private ADXRS450_Gyro gyro;

    //Variables for if robot is field oriented and Rotational2D
    private Rotation2d gyroOffset;
    private boolean fieldOriented;

    //Variable for the "Smoother" Controls
    private final SlewRateLimiter speedLimiter;
    private final SlewRateLimiter turnLimiter;

    //Variables for field and odometry
    private final DifferentialDriveOdometry odometry;
    private final Field2d field;

    public Drivetrain(){
        //Creating Objects for the Motors Controllers
        rightmotor = new VictorSP(RightMotorChannel); 
        leftmotor = new VictorSP(LeftMotorChannel);

        //Setting the right side to inverted to have them move relative to each other
        rightmotor.setInverted(true);
        leftmotor.setInverted(false);

        //bind both motors to Differenital Drive Object
        drive = new DifferentialDrive(leftmotor, rightmotor);
        gyro = new ADXRS450_Gyro(); //Object for the Gyro Sensor

        drive.setSafetyEnabled(true);
        drive.setExpiration(0.1); 

        //Creating Objects of each Variable
        field = new Field2d();
        gyroOffset = new Rotation2d();
        fieldOriented = false; //Setting current status of field oriented to false

        //Object of the Odometry which estimates the robot's position on the field by using sensor data
        odometry = new DifferentialDriveOdometry(getGyroRotation2d(), 0, 0);

        //Used to make the input signals from the joystick smoother
        speedLimiter = new SlewRateLimiter(3.0);
        turnLimiter = new SlewRateLimiter(3.0);

        gyro.calibrate();
        gyro.reset();

    }

    //Applies the deadband and the Limiters, Using the squareInput --> Changes the Input of the Joystick
    public void drive(double speed, double turn, boolean squareInput){
        speed = applyDeadband(speed, 0.02);
        turn = applyDeadband(turn, 0.02);

        speed = speedLimiter.calculate(speed);
        turn = turnLimiter.calculate(turn);

        drive.arcadeDrive(speed, turn, squareInput);
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
        if(!fieldOriented){
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
        return fieldOriented;
    }

    //Toggle between robot-drive and field-oriented drive
    public void toggleFieldOriented(){
        fieldOriented = !fieldOriented;

        if(fieldOriented){
            gyroOffset = getGyroRotation2d();
        }
    }

    //reset fied orientated drive
    public void resetFieldOrientation(){
        gyroOffset = getGyroRotation2d();
    }

    public void resetGyro(){
        gyro.reset();
        gyroOffset = getGyroRotation2d();
    }

    //Get gyro Angle
    public double getGyroAngle(){
        return gyro.getAngle();
    }

    //Converts Gyro Angle to Rotation2D
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
