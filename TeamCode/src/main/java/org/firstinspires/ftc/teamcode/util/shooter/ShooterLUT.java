package org.firstinspires.ftc.teamcode.util.shooter;

import java.util.Map;
import java.util.TreeMap;

public class ShooterLUT {
    // Key: distance, Value: ShotConfig
    private final TreeMap<Double, ShotConfig> table = new TreeMap<>();

    public ShooterLUT() {
        // TODO: add / tune values
        // distance, rpm, hood servo position, turret offset
        table.put(10.0, new ShotConfig(1800, 0.32, 0.0));
        table.put(12.0, new ShotConfig(2000, 0.38, 0.0));
    }

    public ShotConfig getForDistance(double d) {
        if (table.isEmpty()) return new ShotConfig(2000, 0.4, 0.0); // fallback

        Map.Entry<Double, ShotConfig> floor = table.floorEntry(d);
        Map.Entry<Double, ShotConfig> ceil = table.ceilingEntry(d);

        if (floor == null) return ceil.getValue();
        if (ceil == null) return floor.getValue();
        if (floor.getKey().equals(ceil.getKey())) return floor.getValue();

        double d0 = floor.getKey();
        double d1 = ceil.getKey();
        double t = (d - d0) / (d1 - d0);

        return ShotConfig.lerp(floor.getValue(), ceil.getValue(), t);
    }
}
