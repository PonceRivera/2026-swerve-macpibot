package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Gancho2 extends SubsystemBase {

  public enum Posicion {
    Home,
    Manual,
    Release
  }

  private double[] ganchoMatrizPosicion = {
      0.0, 0.0, 200.0,
  };

  private final SparkMax ganchoMotor = new SparkMax(15, MotorType.kBrushless);
  private final RelativeEncoder ganchoEncoder = ganchoMotor.getEncoder();

  private final ProfiledPIDController m_controller_gancho = new ProfiledPIDController(
      0.1, 0.0, 0.0,
      new TrapezoidProfile.Constraints(40, 60));

  private double posicion_gancho = 0.0;
  private Posicion m_gancho_current_position = Posicion.Home;

  public Gancho2() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);
    ganchoMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ganchoEncoder.setPosition(0.0);
    m_controller_gancho.reset(0.0);
  }

  @Override
  public void periodic() {
    posicion_gancho = ganchoEncoder.getPosition();
    SmartDashboard.putNumber("Gancho/Posicion Actual", posicion_gancho);
    SmartDashboard.putNumber("Gancho/Objetivo", m_controller_gancho.getGoal().position);
    SmartDashboard.putString("Gancho/Estado", m_gancho_current_position.name());
  }

  private void setGanchoObjetivo(double posicion) {
    m_controller_gancho.setGoal(posicion);
  }

  private void controlLoop() {
    double output = m_controller_gancho.calculate(posicion_gancho);
    ganchoMotor.set(output);
  }

  private void manualGancho(double input) {
    double scaledValue = (input + (input < 0 ? 0.25 : -0.25)) / (1 - 0.25);
    input = (Math.abs(input) > Math.abs(0.25)) ? scaledValue : 0;

    if (Math.abs(input) < 0.001) {
      return;
    }
    m_gancho_current_position = Posicion.Manual;
    setGanchoObjetivo(posicion_gancho + (input * 10));
  }

  public void stop() {
    ganchoMotor.set(0);
  }

  public Command mandarGanchoAPosicion(Posicion posicion) {
    return runOnce(() -> {
      m_gancho_current_position = posicion;
      setGanchoObjetivo(ganchoMatrizPosicion[posicion.ordinal()]);
    }).andThen(run(this::controlLoop));
  }

  public Command controlLoopCommand(DoubleSupplier raw_gancho) {
    return run(() -> {
      manualGancho(raw_gancho.getAsDouble());
      controlLoop();
    }).finallyDo((interrupted) -> stop());
  }

  public Command subirManualCommand(double input) {
    return run(() -> {
      manualGancho(input);
      controlLoop();
    }).finallyDo((interrupted) -> stop());
  }

  public Command bajarManualCommand(double input) {
    return run(() -> {
      manualGancho(-input);
      controlLoop();
    }).finallyDo((interrupted) -> stop());
  }

  public Command bajarPorTiempoCommand(double segundos) {
    return runOnce(() -> mandarGanchoAPosicion(Posicion.Home)).andThen(Commands.waitSeconds(segundos));
  }

  public Command subirPorTiempoCommand(double segundos) {
    return runOnce(() -> mandarGanchoAPosicion(Posicion.Release)).andThen(Commands.waitSeconds(segundos));
  }
}
