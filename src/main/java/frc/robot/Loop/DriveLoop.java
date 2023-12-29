package frc.robot.Loop;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import frc.robot.Telemetry;
import frc.robot.Subsystem.Drive;
import frc.robot.Telemetry.prefs;
import frc.robot.generated.TunerConstants;
import frc.robot.util.OI;

public class DriveLoop extends Drive {

    private static final double MaxSpeed = 3; // 6 meters per second desired top speed
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

    public static DriveLoop getInstance() {
        if (m_Instance == null) {
            m_Instance = new DriveLoop();
        }
        return m_Instance;
    }

    public void setDriveState(DriveStates state) {
        m_driveState = state;
    }

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
                this.operatorControl();;
                break;
            case DISABLED:
                this.stop();
                break;
        }

    }

    public void operatorControl() {
        super.applyRequest(drive.withVelocityX(-oi.getMappedDriveLeftY()
                        * MaxSpeed)//Preferences.getDouble("OPEN_LOOP_GAIN", prefs.OPEN_LOOP_GAIN))
                .withVelocityY(
                        -oi.getDriveLeftX() * MaxSpeed)//Preferences.getDouble("OPEN_LOOP_GAIN", prefs.OPEN_LOOP_GAIN))
                .withRotationalRate(-oi.getDriveRightX() * MaxAngularRate));
    }

    public void stop() {
        super.applyRequest(drive
                .withVelocityX(0)
                .withVelocityY(
                        0)
                .withRotationalRate(0));
    }
}
