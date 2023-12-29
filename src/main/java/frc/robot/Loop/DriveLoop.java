package frc.robot.Loop;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import frc.robot.Telemetry;
import frc.robot.Subsystem.Drive;
import frc.robot.Telemetry.prefs;
import frc.robot.generated.TunerConstants;
import frc.robot.util.OI;

public class DriveLoop extends Drive {

    private static final double MaxSpeed = 6;
    private static final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private OI oi; // My OI
    private static DriveLoop m_Instance;
    private DriveStates m_driveState;

    public enum DriveStates {
        OPEN_LOOP,
        DISABLED
    }

    /**
     * 
     * @return a single instance of the DriveLoop class
     */
    public static DriveLoop getInstance() {
        if (m_Instance == null) {
            m_Instance = new DriveLoop();
        }
        return m_Instance;
    }

    /**
     * Set the drive state
     * 
     * @param state drive state
     */
    public void setDriveState(DriveStates state) {
        m_driveState = state;
    }

    /**
     * Constructor
     */
    private DriveLoop() {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
                TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        oi = OI.getInstance();
        m_driveState = DriveStates.DISABLED;

        if (Utils.isSimulation()) {
            driveTrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        driveTrain.registerTelemetry(logger::telemeterize);

    }

    @Override
    public void periodic() {
        switch (m_driveState) {
            case OPEN_LOOP:
                this.operatorControl();
                break;
            case DISABLED:
                this.stop();
                this.applyRequest(brake);
                break;
            default:
                break;
        }

    }

    /**
     * Robot drives with operator input
     */
    public void operatorControl() {
        super.applyRequest(drive.withVelocityX(-oi.getMappedDriveLeftY()
                * Preferences.getDouble("OPEN_LOOP_GAIN", prefs.OPEN_LOOP_GAIN))
                .withVelocityY(
                        -oi.getDriveLeftX() * Preferences.getDouble("OPEN_LOOP_GAIN", prefs.OPEN_LOOP_GAIN))
                .withRotationalRate(-oi.getDriveRightX() * MaxAngularRate));
    }

    /**
     * Stop drivetrain
     */
    public void stop() {
        super.applyRequest(drive
                .withVelocityX(0)
                .withVelocityY(
                        0)
                .withRotationalRate(0));
    }
}
