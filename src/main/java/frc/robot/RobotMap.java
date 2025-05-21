package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Subsystem.Drivetrain;

public final class RobotMap {
    public static class OperatorConstants {
        public static final int LEFT_MOTOR_CHANNEL = 0;
        public static final int RIGHT_MOTOR_CHANNEL = 1;

        public static final int JOYSTICK_PORT = 0;

        public static final double SLEW_RATE_LIMITER = 3.0;

        public static final int FIELD_ORIENTED_BUTTON = 4;
        public static final int RESET_FIELD_BUTTON = 5;

        public static final int AIM_BUTTON = 1;
        public static final int TOGGLE_VISION_BUTTON = 3;

        public static final int TOGGLE_CURVATURE_BUTTON = 7;
    }
    public static class OperatorVariables {
        public Drivetrain drivetrain;
        public VictorSP leftMotor;
        public VictorSP rightMotor;

        public DifferentialDrive drive;

        public ADXRS450_Gyro gyro;
        public Rotation2d gyroOffset;
        public boolean fieldOriented;

        public final SlewRateLimiter speedLimiter;
        public final SlewRateLimiter turnLimiter;

        public final DifferentialDriveOdometry odometry;
        public final Field2d field;

        public final PhotonCamera camera;
        public PhotonTrackedTarget currentTarget;
        public boolean hasTarget;
        public boolean isEnabled;

        public boolean visionEnabled;

        public OperatorVariables(){
            drivetrain = new Drivetrain();
            rightMotor = new VictorSP(OperatorConstants.RIGHT_MOTOR_CHANNEL);
            leftMotor = new VictorSP(OperatorConstants.LEFT_MOTOR_CHANNEL);
            drive = new DifferentialDrive(leftMotor, rightMotor);

            rightMotor.setInverted(true);
            leftMotor.setInverted(false);
            drive.setSafetyEnabled(true);
            drive.setExpiration(0.1);

            gyro = new ADXRS450_Gyro();
            gyroOffset = new Rotation2d();
            field = new Field2d();
            odometry = new DifferentialDriveOdometry(drivetrain.getGyroRotation2d(), 0, 0);

            fieldOriented = false;

            speedLimiter = new SlewRateLimiter(OperatorConstants.SLEW_RATE_LIMITER);
            turnLimiter = new SlewRateLimiter(OperatorConstants.SLEW_RATE_LIMITER);

            camera = new PhotonCamera("photovision");
            currentTarget = new PhotonTrackedTarget();
            hasTarget = false;
            isEnabled = true;

            gyro.calibrate();
            gyro.reset();

            visionEnabled = true;
        }
    }

    public static final OperatorVariables variables = new OperatorVariables();
}
