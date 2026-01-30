package org.firstinspires.ftc.teamcode.util.shooter;

import java.util.Map;
import java.util.TreeMap;

public class ShooterLUT {
    // Key: distance, Value: ShotConfig
    private final TreeMap<Double, ShotConfig> table = new TreeMap<>();

    public ShooterLUT() {
        // TODO: add / tune values
        // distance, rpm, hood servo position, turret offset
        table.put(170.0, new ShotConfig(.71, 0.8, 0.0));
        table.put(137.0, new ShotConfig(.63, 0.8, 0.0));
        table.put(93.0, new ShotConfig(.54, 0.8, 0.0));
        table.put(75.0, new ShotConfig(.48, 0.6, 0.0));
        table.put(27.0, new ShotConfig(.4, 0.1, 0.0));
    }

    public ShotConfig getForDistance(double d) {
        if (table.isEmpty()) return new ShotConfig(0, 0.0, 0.0); // fallback

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
