package org.firstinspires.ftc.teamcode.util.shooter;

public class ShotConfig {
    public final double rpm;
    public final double hoodPos;
    public final double turretOffset;

    public ShotConfig(double rpm, double hoodPos, double turretOffset) {
        this.rpm = rpm;
        this.hoodPos = hoodPos;
        this.turretOffset = turretOffset;
    }

    public static ShotConfig lerp(ShotConfig a, ShotConfig b, double t) {
        double rpm = a.rpm + (b.rpm - a.rpm) * t;
        double hood = a.hoodPos + (b.hoodPos - a.hoodPos) * t;
        double turretOffset = a.turretOffset + (b.turretOffset - a.turretOffset) * t;
        return new ShotConfig(rpm, hood, turretOffset);
    }
}
