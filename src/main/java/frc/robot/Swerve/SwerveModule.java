package frc.robot.Swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ModuleConstants;

/**
 * Representa un módulo de swerve drive individual, que controla tanto la 
 * dirección (steering) como la tracción (drive).
 * 
 * @author Juan Felipe Zepeda del Toro
 * @author Fernando Joel Cruz Briones
 * @version 1.1
 */
public class SwerveModule {

    // Motores de conducción (drive) y dirección (steering)
    private TalonFX driveMotor, steerMotor;

    // Control de velocidad y posicion del TalonFX
    private VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);
    private PositionDutyCycle positionControl = new PositionDutyCycle(0);

    // Encoder absoluto para dirección
    private CANcoder absolutNcoder;

    // Configuraciones del encoder absoluto
    private boolean absolutNcoderReversed;
    private double absolutNcoderOffSetRad;

    // Controlador PID para dirección
    @SuppressWarnings("unused")
    private PIDController steeringPidController;

    /**
     * Constructor para inicializar un módulo Swerve.
     *
     * @param driveMotorId          ID del motor de tracción
     * @param steerMotorId          ID del motor de dirección
     * @param driveMotorReversed    Si el motor de tracción está invertido
     * @param steerMotorReversed    Si el motor de dirección está invertido
     * @param CANcoderID            ID del encoder absoluto
     * @param absolutNcoderOffSet   Offset del encoder absoluto en radianes
     * @param CANcoderReversed      Si el encoder absoluto está invertido
     */
    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed,
                        int CANcoderID, double absolutNcoderOffSet, boolean CANcoderReversed) {

        // Inicialización de variables
        this.absolutNcoderOffSetRad = absolutNcoderOffSet;
        this.absolutNcoderReversed = CANcoderReversed;

        // Inicialización de motores y encoder
        absolutNcoder = new CANcoder(CANcoderID, ModuleConstants.CAN_BUS_NAME);
        driveMotor = new TalonFX(driveMotorId, ModuleConstants.CAN_BUS_NAME);
        steerMotor = new TalonFX(steerMotorId, ModuleConstants.CAN_BUS_NAME);

        // Configuración de motores
        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        steerMotor.setNeutralMode(NeutralModeValue.Brake);

        // Configuración de PID (sin Feedforward)
        Slot0Configs slot0Configs = new Slot0Configs()
                .withKP(ModuleConstants.kPDrive)  // Proporcional
                .withKI(ModuleConstants.kIDrive)  // Integral
                .withKD(ModuleConstants.kDDrive); // Derivativo
        
        driveMotor.getConfigurator().apply(slot0Configs);
        steerMotor.getConfigurator().apply(slot0Configs);
                        
        // Reiniciar encoders
        resetEncoders();

        // Parámetros PID dinámicos en SmartDashboard
        SmartDashboard.putNumber("Steering kP", ModuleConstants.kPDrive);
        SmartDashboard.putNumber("Steering kI", ModuleConstants.kIDrive);
        SmartDashboard.putNumber("Steering kD", ModuleConstants.kDDrive);
        SmartDashboard.putNumber("Steering FD", ModuleConstants.kFDrive);
    }

    /**
     * Reinicia los encoders relativos usando la posición absoluta del CANcoder.
     */
    public void resetEncoders() {
        driveMotor.setPosition(0);
        steerMotor.setPosition(getAbsolutePosition());
    }

    /**
     * Devuelve la posición absoluta del encoder en radianes con offset.
     *
     * @return Posición del encoder absoluto en radianes.
     */
    public double getAbsolutePosition() {
        double angle = absolutNcoder.getAbsolutePosition().getValue() * 2.0 * Math.PI;
        angle -= absolutNcoderOffSetRad;
        return angle * (absolutNcoderReversed ? -1 : 1);
    }

    /**
     * Configura el estado deseado del módulo (velocidad y ángulo).
     *
     * @param state Estado deseado del módulo.
     */
    public void setDesiredState(SwerveModuleState state) {
        // Si la velocidad es cercana a cero, detener motores
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Optimizar el estado
        state = SwerveModuleState.optimize(state, getState().angle);

        // Configurar velocidad para el motor Drive con Feedforward
        double desiredVelocityRPM = state.speedMetersPerSecond / ModuleConstants.kDriveEncoderRPM2MeterPerSec;
        double feedForward = ModuleConstants.kFDrive * desiredVelocityRPM; // kF calculado manualmente

        driveMotor.setControl(velocityControl
                .withVelocity(desiredVelocityRPM) // Control de velocidad
                .withFeedForward(feedForward));   // Feedforward manual

        // Configurar posición del motor Steer
        double desiredSteerPosition = state.angle.getRadians();
        steerMotor.setControl(positionControl.withPosition(desiredSteerPosition));
    }

    /**
     * Detiene los motores de tracción y dirección.
     */
    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    /**
     * Obtiene el estado actual del módulo (velocidad y ángulo).
     *
     * @return Estado actual del módulo.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteeringPosition()));
    }

    /**
     * Obtiene la posición del módulo (distancia recorrida y ángulo actual).
     *
     * @return Posición actual del módulo.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteeringPosition()));
    }

    /**
     * Obtiene la posición del motor de dirección en radianes.
     *
     * @return Posición en radianes.
     */
    public double getSteeringPosition() {
        return steerMotor.getPosition().getValue() * ModuleConstants.kSteeringEncoderRot2Rad;
    }

    /**
     * Obtiene la velocidad actual del motor de tracción en m/s.
     *
     * @return Velocidad en metros/segundo.
     */
    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValue() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    /**
     * Obtiene la posición del motor de tracción en metros.
     *
     * @return Posición en metros.
     */
    public double getDrivePosition() {
        return driveMotor.getPosition().getValue() * ModuleConstants.kDriveEncoderRot2Meter;
    }
}
