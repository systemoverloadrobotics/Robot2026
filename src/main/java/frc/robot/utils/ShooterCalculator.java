package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.units.measure.Angle;
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
    private static final double va = 0.0000072671;
    private static final double vb = -0.000756726;
    private static final double vc = 0.0177134;
    private static final double vd = 1.44862;
    private static final double vf = 40.35546;

    // velocity lookup table (In ft/s)
    private static final double[] velocities = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    // regression coefficients for angles
    private static final double aa = 0.0000428106;
    private static final double ab = -0.00536478;
    private static final double ac = 0.24221;
    private static final double ad = -4.79607;
    private static final double af = 85.94505;

    // angles lookup table (In radians)
    private static final double[] angles = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    /**
     * use regression formula to compute linear velocity given distance
     * @param distance distance in feet
     * @return linear velocity
     */
    public static LinearVelocity getRegressionVelocity(Distance distance) {

        double x = distance.in(Feet);
        double v = va * Math.pow(x, 4) + vb * Math.pow(x, 3) + vc * Math.pow(x, 2) + vd * x + vf;
        return FeetPerSecond.of(v);
    }

    /**
     * take distance, half it, if its bigger than/equal to velocities array size (12) then 
     * throw exception, otherwise return the value in velocities array index
     * @param distance distance
     * @return value from lookup array in feet/s
     */
    public static LinearVelocity getLookupTableVelocity(Distance distance) {
        double x = distance.in(Feet);

        int index = Math.abs((int) Math.round(x * 0.5));
        if (index >= velocities.length) {
            throw new IllegalArgumentException("Distance out of bounds for lookup table");
        }
        return FeetPerSecond.of(velocities[index]);
    }

    /**
     * use regression formula to compute angles given distance
     * @param distance distance in feet
     * @return angle in degrees
     */
    public static Angle getRegressionAngle(Distance distance) {

        double x = distance.in(Feet);
        double a = aa * Math.pow(x, 4) + ab * Math.pow(x, 3) + ac * Math.pow(x, 2) + ad * x + af;
        return Degrees.of(a);
    }

    /**
     * take distance, half it, if its bigger than/equal to angles array size (12) then 
     * throw exception, otherwise return the value in angles array index
     * @param distance distance
     * @return value from lookup array in degrees
     */
    public static Angle getLookupAngle(Distance distance) {
        double x = distance.in(Feet);

        int index = Math.abs((int) Math.round(x * 0.5));
        if (index >= angles.length) {
            throw new IllegalArgumentException("Distance out of bounds for lookup table");
        }
        return Degrees.of(angles[index]);
    }
}
