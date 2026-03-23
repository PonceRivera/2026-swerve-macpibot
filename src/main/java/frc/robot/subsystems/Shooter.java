package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public static final int MOTOR_LEFT_CAN_ID = 23;
    public static final int MOTOR_RIGHT_CAN_ID = 16;
    public static final double SHOOT_SPEED = -0.8;
    public static final double SHOOT_SPEED_RIGTH = -0.7;
    public static final double SHOOT_FULL_SPEED = 1.0;

    private static final double SPIN_UP_DELAY = 0.5;
    private static final double SHOOT_FULL_TIMEOUT = 4.0;

    // Configuración Kitbot 2026 (85 grados)
    private static final double DISTANCIA_MIN_METROS = 1.0;
    private static final double POTENCIA_MINima = -0.4;
    private static final double POTENCIA_MAXima = -1.0;
    private static final double PENDIENTE_POTENCIA = -0.15; // Ajuste de potencia por metro extra

    private final SparkMax motorRight = new SparkMax(MOTOR_RIGHT_CAN_ID, MotorType.kBrushless);
    private final TalonFX motorLeft = new TalonFX(MOTOR_LEFT_CAN_ID);
    private final Timer spinUpTimer = new Timer();

    public Shooter() {

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kCoast);
        rightConfig.smartCurrentLimit(60);
        motorRight.configure(rightConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.CurrentLimits.StatorCurrentLimit = 60;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorLeft.getConfigurator().apply(leftConfig);
        motorLeft.setNeutralMode(NeutralModeValue.Coast);

    }

    public void shoot() {
        motorLeft.setControl(new DutyCycleOut(SHOOT_SPEED));
        motorRight.set(SHOOT_SPEED_RIGTH);
    }

    public void shootFull() {
        motorLeft.setControl(new DutyCycleOut(SHOOT_FULL_SPEED));
        motorRight.set(SHOOT_FULL_SPEED);
    }

    public void stop() {
        motorLeft.setControl(new DutyCycleOut(0));
        motorRight.set(0);
    }

    public void dispararSegunDistancia(double distanciaMetros) {
        if (distanciaMetros <= 0) {
            stop();
            return;
        }

        // Formula lineal simple: Potencia = Base + (Distancia - Min) * Pendiente
        double velocidadIzq = POTENCIA_MINima + (distanciaMetros - DISTANCIA_MIN_METROS) * PENDIENTE_POTENCIA;

        // Limitar potencia entre MINIma y MAXima
        velocidadIzq = Math.max(Math.min(velocidadIzq, POTENCIA_MINima), POTENCIA_MAXima);

        double ratioSpin = SHOOT_SPEED_RIGTH / SHOOT_SPEED;
        double velocidadDer = velocidadIzq * ratioSpin;

        motorLeft.setControl(new DutyCycleOut(velocidadIzq));
        motorRight.set(velocidadDer);
    }

    @Override
    public void periodic() {
    }

    public Command shootCommand() {
        return Commands.runEnd(
                () -> {
                    motorRight.set(SHOOT_SPEED_RIGTH);
                    motorLeft.setControl(new DutyCycleOut(spinUpTimer.hasElapsed(SPIN_UP_DELAY) ? SHOOT_SPEED : 0.0));
                },
                () -> {
                    stop();
                    spinUpTimer.stop();
                },
                this).beforeStarting(() -> {
                    spinUpTimer.reset();
                    spinUpTimer.start();
                });
    }

    public Command shootFullCommand() {
        return Commands.run(this::shootFull, this)
                .withTimeout(SHOOT_FULL_TIMEOUT)
                .andThen(Commands.runOnce(this::stop, this));
    }

    public Command dispararSegunDistanciaCommand(DoubleSupplier distanceSupplier) {
        return Commands.runEnd(
                () -> dispararSegunDistancia(distanceSupplier.getAsDouble()),
                this::stop,
                this);
    }
}