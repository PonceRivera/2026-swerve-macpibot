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

    public static final int MOTOR_CAN_ID = 15;
    public static final double SUBIR_SPEED = 0.7;
    public static final double BAJAR_SPEED = -1;
    public static final double MANUAL_SPEED_50 = 1;
    public static final double CONVERSION_FACTOR_POSITION = 1.0 / 75.0;
    public static final double CONVERSION_FACTOR_VELOCITY = CONVERSION_FACTOR_POSITION / 60.0;

    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double GRABAR_SPEED = -0.4;

    private double[] posiciones = { 0.0, 0.5, 0.0 };
    private final String[] posicionesNombres = { "Reposo", "Subir", "Bajar" };

    private final SparkMax motor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    private double currentSetpoint = 0.0;
    private String activePositionName = "Unknown";

    public Gancho() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);
        config.closedLoop
                .p(kP)
                .i(kI)
                .d(kD)
                .outputRange(-0.5, 0.5);
        config.encoder
                .positionConversionFactor(CONVERSION_FACTOR_POSITION)
                .velocityConversionFactor(CONVERSION_FACTOR_VELOCITY);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = motor.getEncoder();
        encoder.setPosition(0.0);
        pidController = motor.getClosedLoopController();
    }

    // --- Movimiento por posición (PID) ---

    public void irAIndice(int index) {
        if (index < 0 || index >= posiciones.length)
            return;
        activePositionName = posicionesNombres[index];
        irAPosicion(posiciones[index]);
    }

    public void irAReposo() {
        irAIndice(0);
    }

    public void irASubir() {
        irAIndice(1);
    }

    public void irABajar() {
        irAIndice(2);
    }

    public void irAPosicion(double posicion) {
        currentSetpoint = posicion;
        pidController.setReference(posicion, ControlType.kPosition);
    }

    // --- Movimiento manual (sin PID) ---

    public void subir() {
        motor.set(SUBIR_SPEED);
    }

    public void bajar() {
        motor.set(BAJAR_SPEED);
    }

    public void stop() {
        motor.set(0);
    }

    public double getPosicion() {
        return encoder.getPosition();
    }

    // --- Commands ---

    public Command irAIndiceCommand(int index) {
        return runOnce(() -> {
            irAIndice(index);
            System.out.println("Gancho: Moviendo a " + activePositionName);
        });
    }

    public Command irAReposoCommand() {
        return runOnce(this::irAReposo);
    }

    public Command irASubirCommand() {
        return runOnce(this::irASubir);
    }

    public Command irABajarCommand() {
        return runOnce(this::irABajar);
    }

    public Command subirCommand() {
        return runEnd(this::subir, this::stop);
    }

    public Command bajarCommand() {
        return runEnd(this::bajar, this::stop);
    }

    public Command moverManualCommand(double speed) {
        return runEnd(
                () -> {
                    motor.set(speed);
                    System.out.println("Gancho Pos: " + getPosicion());
                },
                this::stop);
    }

    public Command subirManualCommand(double speed) {
        return moverManualCommand(speed);
    }

    public Command bajarManualCommand(double speed) {
        return moverManualCommand(-speed);
    }

    public Command moverPorTiempoCommand(double speed, double segundos) {
        return runEnd(() -> motor.set(speed), this::stop).withTimeout(segundos);
    }

    public Command mueveteManual50Command(boolean reverse) {
        double speed = reverse ? -MANUAL_SPEED_50 : MANUAL_SPEED_50;
        return runEnd(() -> motor.set(speed), this::stop);
    }

    public Command grabarDatosCommand() {
        return run(() -> {
            motor.set(GRABAR_SPEED);
            System.out.println("Gancho Posicion Actual: " + getPosicion());
        })
                .beforeStarting(() -> System.out.println("Gancho: Buscando posición..."))
                .finallyDo((interrupted) -> {
                    stop();
                    System.out.println("Gancho: Posición final detenida: " + getPosicion());
                });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gancho/Posicion Actual", getPosicion());
        SmartDashboard.putNumber("Gancho/Objetivo (Setpoint)", currentSetpoint);
        SmartDashboard.putString("Gancho/Nombre Posicion", activePositionName);

        for (int i = 0; i < posiciones.length; i++) {
            String key = "Gancho/Pos " + posicionesNombres[i];
            posiciones[i] = SmartDashboard.getNumber(key, posiciones[i]);
            SmartDashboard.putNumber(key, posiciones[i]);
        }
    }
}
