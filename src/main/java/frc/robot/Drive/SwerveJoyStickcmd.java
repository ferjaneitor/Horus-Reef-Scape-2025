package frc.robot.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Swerve.SwerveSubsystem;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;

/**
 * Comando para controlar el Swerve Drive usando un joystick.
 * Este comando recibe las entradas de velocidad (X, Y, Z) desde un joystick
 * y las traduce en velocidades del chasis para mover el robot con orientación
 * relativa al campo o relativa al robot.
 * 
 * @author Juan Felipe Zepeda del Toro
 * @author Fernando Joel Cruz Briones
 * @version 1.2
 */
public class SwerveJoyStickcmd extends Command {

    // Subsistema Swerve que controlaremos
    private final SwerveSubsystem swerveSubsystem;

    // Proveedores de entrada para las velocidades X, Y y Z
    private final Supplier<Double> xSpdFunction, ySpdFunction, zSpdFunction;

    // Proveedor de entrada para determinar si el robot está orientado al campo
    private final Supplier<Boolean> fieldOrientedSupplier;

    // Limitadores de velocidad para hacer los movimientos más suaves
    private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    // Variable para saber si el robot está en modo orientado al campo
    @SuppressWarnings("unused")
    private boolean isFieldOriented;

    /**
     * Constructor del comando SwerveJoyStickcmd.
     * 
     * @param SwerveSubsystems      El subsistema Swerve que controla los módulos.
     * @param X                     Proveedor de velocidad en el eje X (avance/retroceso).
     * @param Y                     Proveedor de velocidad en el eje Y (lateral).
     * @param Z                     Proveedor de velocidad en el eje Z (rotación).
     * @param FieldOriented         Proveedor booleano que determina si el control es orientado al campo.
     */
    public SwerveJoyStickcmd(
            SwerveSubsystem SwerveSubsystems,
            Supplier<Double> X,
            Supplier<Double> Y,
            Supplier<Double> Z,
            Supplier<Boolean> FieldOriented) {

        // Inicializar el subsistema Swerve
        this.swerveSubsystem = SwerveSubsystems;

        // Inicializar las funciones para capturar entradas del joystick
        this.xSpdFunction = X;
        this.ySpdFunction = Y;
        this.zSpdFunction = Z;

        // Proveedor para la orientación por campo
        this.fieldOrientedSupplier = FieldOriented;

        // Configuración de los limitadores de aceleración
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.zLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        // Requiere el subsistema Swerve
        addRequirements(swerveSubsystem);
    }

    /**
     * Método llamado cuando el comando inicia.
     */
    @Override
    public void initialize() {
        // No requiere inicialización adicional
    }

    /**
     * Método que se ejecuta constantemente mientras el comando está activo.
     */
    @Override
    public void execute() {
        // 1. Obtener las velocidades del joystick
        double xSpeed = xSpdFunction.get();   // Velocidad en el eje X (avance/retroceso)
        double ySpeed = -ySpdFunction.get();  // Velocidad en el eje Y (lateral)
        double zSpeed = -zSpdFunction.get();  // Velocidad en el eje Z (rotación)

        // 2. Aplicar Deadband para evitar movimientos no deseados por ruido
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        zSpeed = Math.abs(zSpeed) > OIConstants.kDeadband ? zSpeed : 0.0;

        // 3. Aplicar limitadores de aceleración para suavizar los movimientos
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        zSpeed = zLimiter.calculate(zSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construir las velocidades del chasis usando orientación relativa al campo o al robot
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedSupplier.get()) {
            // Movimiento relativo al campo
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, zSpeed, swerveSubsystem.getRotation2d());
            isFieldOriented = true;
        } else {
            // Movimiento relativo al robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
            isFieldOriented = false;
        }

        // 5. Pasar las velocidades del chasis al subsistema Swerve
        swerveSubsystem.drive(chassisSpeeds);
    }

    /**
     * Método llamado cuando el comando termina o es interrumpido.
     * 
     * @param interrupted Si el comando fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        // Detener los módulos del Swerve Drive
        swerveSubsystem.stopModules();
    }

    /**
     * Determina si el comando ha finalizado.
     * 
     * @return Siempre retorna false ya que este comando debe ejecutarse continuamente.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
