package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/*
 * @author Juan Felipe Zepeda del Toro
 * @author Fernando Joel Cruz Briones
 * @version 1.1
 */

public class constants {
    public static final class ModuleConstants {
    
        //Diametro de la llanta
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //(MODIFY)

        //Drive Gear Ratio
        public static final double kDriveMotorGearRatio = 1 / 8.14; //(MODIFY)

        //Steering Gear Ratio
        public static final double kTurningMotorGearRatio = 1 / 21.4286; //(MODIFY)

        //Configuraciones para los encoders, se pasan de rotaciones a metros para el drive motor y de rotaciones a grados para el steering motor
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kSteeringEncoderRot2Rad = kTurningMotorGearRatio * Math.PI * 2;

        // Configuraciones para sacar las revoluciones por metros y su aceleracion
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kSteeringEncoderRPM2RadPerSec = kSteeringEncoderRot2Rad / 60;

        //Configuracion del PIDF de CTRElectronis para los motores TalonFX
        public static final double kPDrive = 0.5 ;
        public static final double kIDrive = 0.0 ;
        public static final double kDDrive = 0.0 ;
        public static final double kFDrive = 0.02 ;

        public static final double kPSteering = 0.5 ;
        public static final double kISteering = 0.0 ;
        public static final double kDSteering = 0.0 ;
        public static final double kFSteering = 0.02 ;
        
        //Nombre del BUS de la CANivore
        public static String CAN_BUS_NAME = "6348 Horus CANivore";
        
    }

    public static final class DriveConstants {

        //Es la velocidad limite que tiene nuestro robot
        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(12.5);

        //La velocidad maxima angular
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;

        //Limitaciones de velocidad para el teleoperado
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.80; // Velocidad lineal maxima
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2; // Velocidad angular maxima
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3; //Acceleracion Lineal Maxima
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;//Aceleracion Maxima Angular

        // La distancia entre las llantas de la derecha y de la izquierda
        public static final double kTrackWidth = Units.inchesToMeters(20.5); //(MODIFY)

        // La distancia entre las llantas delanteras y las traceras
        public static final double kWheelBase = Units.inchesToMeters(20.5); //(MODIFY)

        //Kinematicas
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        //Enfrente Izquierda

        //Drive Motor
        public static final int kFrontLeftDriveMotorPort = 9;
        public static final boolean kFrontLeftDriveEncoderReversed = true;

        //Steering Motor
        public static final int kFrontLeftSteeringMotorPort = 5;
        public static final boolean kFrontLeftSteeringEncoderReversed = false;

        //Absolute Encoder
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1 ;
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.076904 ;

        //Enfrente Derecha

        //Drive Motor
        public static final int kFrontRightDriveMotorPort = 10;
        public static final boolean kFrontRightDriveEncoderReversed = false ;

        //Steering Motor
        public static final int kFrontRightSteeringMotorPort = 6 ;
        public static final boolean kFrontRightSteeringEncoderReversed = false ;

        //Absolute Encoder
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2 ;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.699707 ;

        //Atras Izquieda

        //Drive Motor
        public static final int kBackLeftDriveMotorPort = 11 ;
        public static final boolean kBackLeftDriveEncoderReversed = true ;

        //Steering Motor
        public static final int kBackLeftSteeringMotorPort = 7 ;
        public static final boolean kBackLeftSteeringEncoderReversed = false ;

        //Absolute Encoder
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3 ;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.987305 ;

        //Atras Derecha

        //Drive Motor
        public static final int kBackRightDriveMotorPort = 12 ;
        public static final boolean kBackRightDriveEncoderReversed = false ;

        //Steering Motor
        public static final int kBackRightSteeringMotorPort = 8 ;
        public static final boolean kBackRightSteeringEncoderReversed = true ;

        //Absolute Encoder
        public static final int kBackRightDriveAbsoluteEncoderPort = 4 ;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.150635 ;

    }

    public static final class AutoConstants {

        //Velocidad Maxima lineal para el AUto
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;

        //Velocidad Maxima Angular para el Auto
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;

        //Aceleracion Maxima lineal para el Auto
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; // 3

        //Aceleracion Maxima Angular para el Auto
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

        //Configuracion del PID
        public static final double kPXController = 0.5;
        public static final double kPYController = 0.5;
        public static final double kPThetaController = 0.05; //3

        //Configuracion de color de alianza
        public static final int kColorTeam = 1; // 1 for RED, -1 for BLUE
        

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {

        //Puerto del control del driver
        public static final int kDriverControllerPort = 0;

        //Puerto del contro de los aditamentos
        public static final int kAddOnsControllerPort = 1;

        //Configuracion los puertos para habilitar mobilidad
        public static final int kDriverYAxis = 1; //joystick Izquierdo en eje Y 
        public static final int kDriverXAxis = 0; //Joystick Izquierdo en eje X
        public static final int kDriverZAxis = 4; //Joystick Derecho en Eje X
        public static final int kDriverAutoTargetButtonIdx = 3;
        public static final int kDriverFieldOrientedButtonIdx = 6; //Boton para orientacion de campo

        public static final double kDeadband = 0.1;//Es la tolerancia para evitar stick drifting
    }

    //Constantes de la LimeLight
    public static final class LimeConstants {

        public static final double cameraHeight = 20; 
        public static final double cameraAngle = 20.5;

        //La distancia del Speaker que debe que estar el robot
        public static final double kDistanceToSpeaker = 70;

        //La distnacia del robot al Amplificador
        public static final double kDistanceToAmp = 45;

        //La altura de la April Tag del Speaker
        public static final double kTargetSpeakertHeight = 54; 

        //La latura de la April Tag del Amp
        public static final double kTargetAmpHeight = 51.625; // Modiffy 58.375      
    }
}
