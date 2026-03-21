package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.cameraserver.CameraServer;

public class Limelight extends SubsystemBase {

    private final NetworkTable table;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry tv;
    private final NetworkTableEntry tid;

    private static final double ALTURA_LENTE_METROS = Units.inchesToMeters(24.5);
    private static final double ALTURA_OBJETIVO_METROS = 1.45; // TODO: Confirmar altura del objetivo
    private static final double ANGULO_MONTAJE_GRADOS = 0.0;     // Camara paralela al piso

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        tid = table.getEntry("tid");

        try {
            String[] urls = {
                    "http://limelight.local:5800/stream.mjpg",
                    "http://10.82.90.11:5800/stream.mjpg"
            };
            HttpCamera limelightFeed = new HttpCamera("Limelight", urls, HttpCameraKind.kMJPGStreamer);
            CameraServer.addCamera(limelightFeed);
            Shuffleboard.getTab("Limelight").add("Cámara", limelightFeed).withSize(4, 3);
        } catch (Exception e) {
            System.out.println("No se pudo agregar Limelight a Shuffleboard: " + e.getMessage());
        }
    }

    public boolean tieneObjetivo() {
        return tv.getDouble(0.0) == 1.0;
    }

    public int getTagID() {
        if (tieneObjetivo()) {
            return (int) tid.getDouble(-1.0);
        }
        return -1;
    }

    public double getXOffset() {
        return tx.getDouble(0.0);
    }

    public double getYOffset() {
        return ty.getDouble(0.0);
    }

    public double getDistanciaMetros() {
        if (!tieneObjetivo())
            return 0.0;

        double anguloHaciaObjetivoRadianes = Math.toRadians(ANGULO_MONTAJE_GRADOS + getYOffset());
        double diferenciaAltura = ALTURA_OBJETIVO_METROS - ALTURA_LENTE_METROS;

        return diferenciaAltura / Math.tan(anguloHaciaObjetivoRadianes);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight/Tiene Objetivo", tieneObjetivo());
        SmartDashboard.putNumber("Limelight/ID del AprilTag", getTagID());
        SmartDashboard.putNumber("Limelight/Angulo X", getXOffset());
        SmartDashboard.putNumber("Limelight/Angulo Y", getYOffset());
        SmartDashboard.putNumber("Limelight/Distancia (m)", getDistanciaMetros());
    }
}