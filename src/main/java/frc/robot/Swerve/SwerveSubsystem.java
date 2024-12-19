package frc.robot.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

/**
 * Subsystem encargado del control del Swerve Drive.
 * Realiza funciones como odometría, actualización de estados y alineación a objetivos.
 * 
 * @author Juan Felipe Zepeda del Toro
 * @author Fernando Joel Cruz Briones
 * @version 1.2
 */
public class SwerveSubsystem extends SubsystemBase {

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Módulos Swerve
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftSteeringMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed, DriveConstants.kFrontLeftSteeringEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightSteeringMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed, DriveConstants.kFrontRightSteeringEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort, DriveConstants.kBackLeftSteeringMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed, DriveConstants.kBackLeftSteeringEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort, DriveConstants.kBackRightSteeringMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed, DriveConstants.kBackRightSteeringEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private SwerveModulePosition[] modulePositions;
    private final SwerveDriveOdometry odometer;

    /** Constructor: inicializa módulos y odometría. */
    public SwerveSubsystem() {
        updateModulePositions();

        odometer = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics,
                getRotation2d(),
                modulePositions);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }).start();
    }

    /** Actualiza las posiciones de los módulos. */
    private void updateModulePositions() {
        modulePositions = new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    /** Reinicia la orientación del gyro a 0 grados. */
    public void zeroHeading() {
        gyro.reset();
    }

    /** Devuelve el ángulo actual del gyro. */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    /** Devuelve la rotación actual como Rotation2d. */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /** Devuelve la posición actual del robot según la odometría. */
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    /** Reinicia la odometría a una nueva posición. */
    public void resetOdometry(Pose2d pose) {
        updateModulePositions();
        odometer.resetPosition(getRotation2d(), modulePositions, pose);
    }

    /** Actualiza periódicamente la odometría y envía datos al SmartDashboard. */
    @Override
    public void periodic() {
        updateModulePositions();
        odometer.update(getRotation2d(), modulePositions);

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    /** Detiene todos los módulos Swerve. */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Convierte las velocidades del chasis a estados individuales de los módulos y los aplica.
     * 
     * @param chassisSpeeds Velocidades deseadas del chasis (avance, lateral y rotación).
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
        // Optimizar velocidades de las ruedas y asignar estados deseados
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }
    

    /**
     * Asigna los estados deseados a los módulos Swerve.
     * 
     * @param desiredStates Estados de los módulos.
     */
    public void setModulesState(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    
    public void resetEncoders(){    

        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();

    }
}
