// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gancho extends SubsystemBase {

    private double[] ganchoMatrizPosicion = { 0.0, 0.5, 0.0 };

    private String[] ganchoPosicionesNombres = { "Reposo", "Subir", "Bajar" };

    public static final int MOTOR_CAN_ID = 15;

    // Ganancias PID (ajustar en robot real)
    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Velocidades manuales (override sin PID, para pruebas)
    public static final double SUBIR_SPEED = 0.3;
    public static final double BAJAR_SPEED = -0.3;
    public static final double MANUAL_SPEED_50 = 0.5;


    public static final double CONVERSION_FACTOR_POSITION = 1.0 / 75.0;
    public static final double CONVERSION_FACTOR_VELOCITY = CONVERSION_FACTOR_POSITION / 60.0;

    private final SparkMax motor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    private double currentSetpoint = 0.0;
    private String activePositionName = "Unknown";

    public Gancho() {
        SparkMaxConfig config = new SparkMaxConfig();

        // Motor en freno: mantiene la posición cuando el PID para
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        // PID — usa el encoder relativo interno del NEO por defecto
        config.closedLoop
                .p(kP)
                .i(kI)
                .d(kD)
                .outputRange(-0.5, 0.5); // Limita la salida del PID para no golpear los topes

        // Configurar el factor de conversión para que todo el PID y .getPosition()
        // trabajen en rotaciones reales del eje de salida del gancho, no del motor.
        config.encoder
                .positionConversionFactor(CONVERSION_FACTOR_POSITION)
                .velocityConversionFactor(CONVERSION_FACTOR_VELOCITY);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        encoder.setPosition(0.0); // Guardamos la posición actual al encender como "Home" (0)

        pidController = motor.getClosedLoopController();
    }

    public void irAReposo() {
        activePositionName = ganchoPosicionesNombres[0];
        irAPosicion(ganchoMatrizPosicion[0]);
    }

    /** Mueve el gancho Arriba — para colgarse (índice 1). */
    public void irASubir() {
        activePositionName = ganchoPosicionesNombres[1];
        irAPosicion(ganchoMatrizPosicion[1]);
    }

    /** Mueve el gancho Abajo — bajar después de subir (índice 2). */
    public void irABajar() {
        activePositionName = ganchoPosicionesNombres[2];
        irAPosicion(ganchoMatrizPosicion[2]);
    }

    /** Mueve el gancho a una posición arbitraria en rotaciones. */
    public void irAPosicion(double posicion) {
        currentSetpoint = posicion;
        pidController.setReference(posicion, ControlType.kPosition);
    }

    /** Sube el gancho a velocidad fija. */
    public void subir() {
        motor.set(SUBIR_SPEED);
    }

    /** Baja el gancho a velocidad fija. */
    public void bajar() {
        motor.set(BAJAR_SPEED);
    }

    /** Detiene el motor. */
    public void stop() {
        motor.set(0);
    }

    /** Posición actual del encoder interno del NEO (rotaciones relativas). */
    public double getPosicion() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        // El dato más importante: la posición actual del encoder (Rotaciones reales del
        // gancho)
        SmartDashboard.putNumber("Gancho/Posicion Actual", getPosicion());

        // Información de control
        SmartDashboard.putNumber("Gancho/Objetivo (Setpoint)", currentSetpoint);
        SmartDashboard.putString("Gancho/Nombre Posicion", activePositionName);

        // -- ACTUALIZACIÓN DESDE EL SMARTDASHBOARD --
        // Aquí "lee" lo que escribas en el SmartDashboard. Si no hay nada escrito,
        // usa el valor que ya tenía guardado en la memoria (ganchoMatrizPosicion).
        ganchoMatrizPosicion[0] = SmartDashboard.getNumber("Gancho/Pos Reposo", ganchoMatrizPosicion[0]);
        ganchoMatrizPosicion[1] = SmartDashboard.getNumber("Gancho/Pos Subir", ganchoMatrizPosicion[1]);
        ganchoMatrizPosicion[2] = SmartDashboard.getNumber("Gancho/Pos Bajar", ganchoMatrizPosicion[2]);

        // Y aquí vuelve a "publicar" el valor actual a la pantalla para confirmarlo
        // visualmente
        SmartDashboard.putNumber("Gancho/Pos Reposo", ganchoMatrizPosicion[0]);
        SmartDashboard.putNumber("Gancho/Pos Subir", ganchoMatrizPosicion[1]);
        SmartDashboard.putNumber("Gancho/Pos Bajar", ganchoMatrizPosicion[2]);
    }

    /** Ejecuta una vez: gancho a Reposo (abajo, posición natural). */
    public Command irAReposoCommand() {
        return runOnce(this::irAReposo);
    }

    /** Ejecuta una vez: gancho Arriba (para colgarse). */
    public Command irASubirCommand() {
        return runOnce(this::irASubir);
    }

    /** Ejecuta una vez: gancho Abajo (bajar después de subir). */
    public Command irABajarCommand() {
        return runOnce(this::irABajar);
    }

    /**
     * Mientras se mantiene: sube el gancho en modo manual (sin PID). Al soltar:
     * para.
     */
    public Command subirCommand() {
        return runEnd(this::subir, this::stop);
    }

    /**
     * Mientras se mantiene: baja el gancho en modo manual (sin PID). Al soltar:
     * para.
     */
    public Command bajarCommand() {
        return runEnd(this::bajar, this::stop);
    }

    /**
     * Sube el gancho a velocidad configurable mientras se mantiene. Imprime
     * posición.
     */
    public Command subirManualCommand(double speed) {
        return runEnd(
                () -> {
                    motor.set(speed);
                    System.out.println("Gancho Pos: " + getPosicion());
                },
                this::stop);
    }

    /**
     * Baja el gancho a velocidad configurable mientras se mantiene. Imprime
     * posición.
     */
    public Command bajarManualCommand(double speed) {
        return runEnd(
                () -> {
                    motor.set(-speed);
                    System.out.println("Gancho Pos: " + getPosicion());
                },
                this::stop);
    }

    /**
     * Mueve el gancho a 0.5 de velocidad (50% poder) mientras se mantiene.
     * Al soltar se detiene.
     */
    public Command mueveteManual50Command(boolean reverse) {
        double speed = reverse ? -MANUAL_SPEED_50 : MANUAL_SPEED_50;
        return runEnd(() -> motor.set(speed), this::stop);
    }

    /**
     * Comando para buscar valores: mueve el gancho a 0.4 de velocidad e imprime
     * la posición actual por consola. Úsalo para llenar ganchoMatrizPosicion.
     */
    public Command grabarDatosCommand() {
        return run(() -> {
            motor.set(-0.4); // Baja el gancho a 0.4 de velocidad
            System.out.println("Gancho Posicion Actual: " + getPosicion());
        })
                .beforeStarting(() -> System.out.println("Gancho: Buscando posición..."))
                .finallyDo((interrupted) -> {
                    stop();
                    System.out.println("Gancho: Posición final detenida: " + getPosicion());
                });
    }

    /**
     * Mueve el gancho a una posición del arreglo según su índice.
     */
    public Command irAIndiceCommand(int index) {
        return runOnce(() -> {
            if (index >= 0 && index < ganchoMatrizPosicion.length) {
                activePositionName = ganchoPosicionesNombres[index];
                irAPosicion(ganchoMatrizPosicion[index]);
                System.out.println("Gancho: Moviendo a " + ganchoPosicionesNombres[index]);
            }
        });
    }
}
