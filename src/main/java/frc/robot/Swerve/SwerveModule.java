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
 * @author Juan Felipe Zepeda del Toro
 * @author Fernando Joel Cruz Briones
 * @version 1.8
 */
public class SwerveModule {

    // Motores de conducción (drive) y dirección (steering)
    private TalonFX driveMotor, steerMotor;

    // Control de velocidad y posición
    private VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);
    private PositionDutyCycle positionControl = new PositionDutyCycle(0);

    // Encoder absoluto para dirección
    private CANcoder absolutNcoder;

    // Configuraciones del encoder absoluto
    private boolean absolutNcoderReversed;
    private double absolutNcoderOffSetRad;

    /**
     * Constructor para inicializar un módulo Swerve.
     */
    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed,
                    int CANcoderID, double absolutNcoderOffSet, boolean CANcoderReversed) {

    this.absolutNcoderOffSetRad = absolutNcoderOffSet;
    this.absolutNcoderReversed = CANcoderReversed;

    // Inicialización de encoders y motores
    absolutNcoder = new CANcoder(CANcoderID, ModuleConstants.CAN_BUS_NAME);
    driveMotor = new TalonFX(driveMotorId, ModuleConstants.CAN_BUS_NAME);
    steerMotor = new TalonFX(steerMotorId, ModuleConstants.CAN_BUS_NAME);

    // Configuración básica de los motores
    driveMotor.setInverted(driveMotorReversed);
    steerMotor.setInverted(steerMotorReversed);
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    steerMotor.setNeutralMode(NeutralModeValue.Brake);

    // Configuración de PID en Phoenix 6
    Slot0Configs driveSlot0Configs = new Slot0Configs();
    driveSlot0Configs.kP = ModuleConstants.kPDrive;
    driveSlot0Configs.kI = ModuleConstants.kIDrive;
    driveSlot0Configs.kD = ModuleConstants.kDDrive;

    Slot0Configs steerSlot0Configs = new Slot0Configs();
    steerSlot0Configs.kP = ModuleConstants.kPSteering;
    steerSlot0Configs.kI = ModuleConstants.kISteering;
    steerSlot0Configs.kD = ModuleConstants.kDSteering;

    // Aplicar configuraciones PID a los motores
    driveMotor.getConfigurator().apply(driveSlot0Configs);
    steerMotor.getConfigurator().apply(steerSlot0Configs);

    // Reiniciar encoders
    resetEncoders();
}


    /**
     * Reinicia los encoders relativos usando la posición absoluta del CANcoder.
     */
    public void resetEncoders() {
        driveMotor.setPosition(0);
        steerMotor.setPosition(getAbsolutePosition()); // Usa el valor del encoder absoluto
    }    

    /**
     * Devuelve la posición absoluta del encoder en radianes con offset.
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
        // Obtener el estado optimizado usando el ángulo actual del módulo
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
    
        // Configurar velocidad del driveMotor con Feedforward
        double desiredVelocityRPM = optimizedState.speedMetersPerSecond / ModuleConstants.kDriveEncoderRPM2MeterPerSec;
        double feedForward = ModuleConstants.kFDrive * desiredVelocityRPM;
    
        driveMotor.setControl(velocityControl
                .withVelocity(desiredVelocityRPM) // Control de velocidad
                .withFeedForward(feedForward));   // Feedforward manual
    
        // Configurar posición del steerMotor usando el ángulo optimizado
        steerMotor.setControl(positionControl.withPosition(optimizedState.angle.getRadians()));
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
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteeringPosition()));
    }    

    /**
     * Obtiene la posición del módulo (distancia recorrida y ángulo actual).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteeringPosition()));
    }

    /**
     * Obtiene la posición del motor de dirección en radianes.
     */
    public double getSteeringPosition() {
        return steerMotor.getPosition().getValue() * ModuleConstants.kSteeringEncoderRot2Rad;
    }

    /**
     * Obtiene la velocidad actual del motor de tracción en m/s.
     */
    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValue() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    /**
     * Obtiene la posición del motor de tracción en metros.
     */
    public double getDrivePosition() {
        return driveMotor.getPosition().getValue() * ModuleConstants.kDriveEncoderRot2Meter;
    }
}
