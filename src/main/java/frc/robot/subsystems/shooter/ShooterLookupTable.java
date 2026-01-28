package frc.robot.subsystems.shooter;

import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * Lookup table that maps target distance (meters) to shot parameters (RPM + angle offset).
 * 
 * Uses linear interpolation between known data points.
 * Values outside the table range will clamp to the nearest endpoint.
 * 
 * TUNING GUIDE:
 * 1. Start with physics-based estimates
 * 2. Test at known distances on the field
 * 3. Record actual RPM and angle that work
 * 4. Update this table with measured values
 */
public class ShooterLookupTable {
    
    // TreeMap automatically sorts by distance and allows efficient lookup
    private final NavigableMap<Double, ShotParameters> table = new TreeMap<>();

    /**
     * Creates the lookup table with pre-configured values.
     * 
     * NOTE: These are INITIAL ESTIMATES. You MUST tune these on the actual field!
     * The table maps: distance (meters) → {flywheelRPM, hoodAngleOffset}
     */
    public ShooterLookupTable() {
        // ============================================================
        // DISTANCE-BASED SHOT PARAMETERS
        // ============================================================
        // Format: addEntry(distanceMeters, flywheelRPM, hoodAngleOffset)
        //
        // hoodAngleOffset is relative to the direction's base angle:
        // - LEFT base: 30°  → actual angle = 30° + offset (valid: 15°-45°)
        // - RIGHT base: 150° → actual angle = 150° + offset (valid: 135°-165°)
        //
        // So offsets should typically be in range [-15°, +15°]
        // ============================================================
        
        // Close range shots (1-2 meters)
        addEntry(1.0, 2000, -5.0);   // 1m: Low RPM, slight inward angle
        addEntry(1.5, 2300, -2.0);   // 1.5m
        addEntry(2.0, 2600, 0.0);    // 2m: Base angle
        
        // Medium range shots (2.5-4 meters)
        addEntry(2.5, 2900, 2.0);    // 2.5m
        addEntry(3.0, 3200, 4.0);    // 3m
        addEntry(3.5, 3500, 6.0);    // 3.5m
        addEntry(4.0, 3800, 8.0);    // 4m
        
        // Long range shots (4.5-6 meters)
        addEntry(4.5, 4100, 10.0);   // 4.5m
        addEntry(5.0, 4400, 12.0);   // 5m
        addEntry(5.5, 4700, 13.0);   // 5.5m
        addEntry(6.0, 5000, 14.0);   // 6m: Near max range
        
        // Maximum range (6+ meters) - use with caution
        addEntry(7.0, 5500, 15.0);   // 7m: Max offset
    }

    /**
     * Adds an entry to the lookup table.
     * 
     * @param distanceMeters Distance to target in meters
     * @param flywheelRPM Target flywheel RPM
     * @param hoodAngleOffset Hood angle offset from base (degrees)
     */
    public void addEntry(double distanceMeters, double flywheelRPM, double hoodAngleOffset) {
        table.put(distanceMeters, new ShotParameters(flywheelRPM, hoodAngleOffset));
    }

    /**
     * Gets shot parameters for a given distance using linear interpolation.
     * 
     * @param distanceMeters Target distance in meters
     * @return Interpolated shot parameters
     */
    public ShotParameters getParameters(double distanceMeters) {
        if (table.isEmpty()) {
            // Fallback if table is empty (should never happen)
            return new ShotParameters(3000, 0);
        }

        // Get the entries immediately below and above the target distance
        var floorEntry = table.floorEntry(distanceMeters);
        var ceilingEntry = table.ceilingEntry(distanceMeters);

        // Handle edge cases (distance outside table range)
        if (floorEntry == null) {
            // Distance is below minimum - use minimum entry
            return ceilingEntry.getValue();
        }
        if (ceilingEntry == null) {
            // Distance is above maximum - use maximum entry
            return floorEntry.getValue();
        }
        if (floorEntry.getKey().equals(ceilingEntry.getKey())) {
            // Exact match found
            return floorEntry.getValue();
        }

        // Linear interpolation between floor and ceiling
        double t = (distanceMeters - floorEntry.getKey()) 
                 / (ceilingEntry.getKey() - floorEntry.getKey());
        
        return ShotParameters.interpolate(floorEntry.getValue(), ceilingEntry.getValue(), t);
    }

    /**
     * Returns the minimum distance in the table.
     */
    public double getMinDistance() {
        return table.isEmpty() ? 0 : table.firstKey();
    }

    /**
     * Returns the maximum distance in the table.
     */
    public double getMaxDistance() {
        return table.isEmpty() ? 0 : table.lastKey();
    }

    /**
     * Returns true if the distance is within the reliable range of the table.
     */
    public boolean isDistanceInRange(double distanceMeters) {
        return distanceMeters >= getMinDistance() && distanceMeters <= getMaxDistance();
    }
}
