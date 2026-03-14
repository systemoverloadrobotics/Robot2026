package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * reference
 * desmos: https://www.desmos.com/calculator/wune2ctiat
 * class containing methods to do shooter calculations that return velocity and
 * angle given distance
 * angle should be between 15 and 45 degrees
 * 
 */

public class ShooterCalculator {

    // regression coefficients for velocity
    // private static final double va = 0.00000371529;
    // private static final double vb = -0.000386874;
    // private static final double vc = 0.00905592;
    // private static final double vd = 0.740602;
    // private static final double vf = 20.63163;

    // velocity lookup table (In ft/s)
    private static final double[] velocities = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    // regression coefficients for angles
    private static final double va = -0.000700283;
    private static final double vb = 0.0300222;
    private static final double vc = -0.41232;
    private static final double vd = 2.6256;
    private static final double vf = 13.20764;

    // angles lookup table (In radians)
    private static final double[] angles = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    /**
     * use regression formula to compute linear velocity given distance
     * @param distance distance in feet
     * @return linear velocity
     */
    // public static LinearVelocity getRegressionVelocity(Distance distance) {

    //     double x = distance.in(Feet);
    //     double v = va * Math.pow(x, 4) + vb * Math.pow(x, 3) + vc * Math.pow(x, 2) + vd * x + vf;
    //     return FeetPerSecond.of(clamp(v, 0, 70));
    // }

    public static AngularVelocity getRegressionVelocity(Distance distance) {

        double x = distance.in(Feet);
        double v = va * Math.pow(x, 4) + vb * Math.pow(x, 3) + vc * Math.pow(x, 2) + vd * x + vf;
        return RotationsPerSecond.of(clamp(v, 0, 70));
    }

    /**
     * take distance, half it, if its bigger than/equal to velocities array size (12) then 
     * throw exception, otherwise return the value in velocities array index
     * @param distance distance
     * @return value from lookup array in feet/s
     */
    // public static LinearVelocity getLookupTableVelocity(Distance distance) {
    //     double x = distance.in(Feet);

    //     int index = Math.abs((int) Math.round(x * 0.5));
    //     if (index >= velocities.length) {
    //         // throw new IllegalArgumentException("Distance out of bounds for lookup table");
    //         index = velocities.length - 1; // Use the last value in the table for out-of-bounds distances
    //     }
    //     return FeetPerSecond.of(velocities[index]);
    // }

    /**
     * use regression formula to compute angles given distance
     * @param distance distance in feet
     * @return angle in degrees
     */
    public static Angle getRegressionAngle(Distance distance) {

        // double x = distance.in(Feet);
        // double a = aa * Math.pow(x, 4) + ab * Math.pow(x, 3) + ac * Math.pow(x, 2) + ad * x + af;
        // return Degrees.of(clamp(a, 45, 75));
        return Degrees.of(2);
    }

    /**
     * take distance, half it, if its bigger than/equal to angles array size (12) then 
     * throw exception, otherwise return the value in angles array index
     * @param distance distance
     * @return value from lookup array in degrees
     */
    // public static Angle getLookupAngle(Distance distance) {
    //     double x = distance.in(Feet);

    //     int index = Math.abs((int) Math.round(x * 0.5));
    //     if (index >= angles.length) {
    //         // throw new IllegalArgumentException("Distance out of bounds for lookup table");
    //         index = angles.length - 1; // Use the last value in the table for out-of-bounds distances
    //     }
    //     return Degrees.of(angles[index]);
    // }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
