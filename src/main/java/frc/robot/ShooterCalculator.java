package frc.robot;
 /** 
  * reference
  * desmos: https://www.desmos.com/calculator/wune2ctiat
  * class containing methods to do shooter calculations that return velocity and angle given distance
  * angle should be between 15 and 45 degrees
  * 
  */ 

public class ShooterCalculator {

    // desmos: gravity = -32.174 f/s^2
    private static final double GRAVITY = -32.174;

    // TODO: change min and max later to correct values
    // time 't' lower and upper bound 
    private static final double TIME_MIN = 0.05;
    private static final double TIME_MAX = 3.00;

    // result containing optimal time, velocity and angle between 15 and 45

    public static class ShooterValues {
        public final double distanceFt;
        public final double timeSec;
        public final double vxFtPerSec;
        public final double vyFtPerSec;
        public final double velocityFtPerSec;
        public final double angleDeg;

        public ShooterValues(double distanceFt,
                          double timeSec,
                          double vxFtPerSec,
                          double vyFtPerSec,
                          double velocityFtPerSec,
                          double angleDeg) {
            this.distanceFt = distanceFt;
            this.timeSec = timeSec;
            this.vxFtPerSec = vxFtPerSec;
            this.vyFtPerSec = vyFtPerSec;
            this.velocityFtPerSec = velocityFtPerSec;
            this.angleDeg = angleDeg;
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder(); 
            sb.append("ShotResult{xFt = "); 
            sb.append(String.format("%.3f", distanceFt));
            sb.append(" ft, "); 
            sb.append("timeSec = "); 
            sb.append(String.format("%.3f", timeSec));
             sb.append(" s, ");sb.append("velocityFtPerSec = "); 
             sb.append(String.format("%.2f", velocityFtPerSec)); 
             sb.append(" ft/s, ");
             sb.append("angleDef = "); 
             sb.append(String.format("%.2f", angleDeg)); 
             sb.append(" deg}"); 
             return sb.toString();
        }
    }

    /**
     * Lookup table with 12 entries
     */

    // 12 entry lookup table with each entry covering 2-ft
    private static final int LOOKUP_TABLE_SIZE = 12;

    private static final double LOOKUP_ENTRY_FT = 2.0;

    public static class LookupTableEntry {
        public double timeOffsetSec;            // seconds
        public double velocityOffsetFtPerSec;   // ft/s
        public double angleOffsetDeg;           // degrees

        public LookupTableEntry(double timeOffsetSec,
                                double velocityOffsetFtPerSec,
                                double angleOffsetDeg) {
            this.timeOffsetSec = timeOffsetSec;
            this.velocityOffsetFtPerSec = velocityOffsetFtPerSec;
            this.angleOffsetDeg = angleOffsetDeg;
        }
    }

    // create and initialize lookup table once when class initializes
    private static final LookupTableEntry[] LOOKUP_TABLE = initLookupTable();

    private static LookupTableEntry[] initLookupTable() {
        LookupTableEntry[] lookupTableEntry = new LookupTableEntry[LOOKUP_TABLE_SIZE];
        for (int i = 0; i < LOOKUP_TABLE_SIZE; i++) {
            lookupTableEntry[i] = new LookupTableEntry(
                    0.0,
                    0.0,
                    0.0);
        }
        return lookupTableEntry;
    }
    /**
     * take distance divide by 2 to get index that has tuning values
     * @param distanceFt
     * @return index
     * 
     * 
     */
    private static int distanceToIndex(double distanceFt) {
        int idx = (int) Math.round(distanceFt / LOOKUP_ENTRY_FT);
        idx = (int) clamp(idx, 0, LOOKUP_TABLE_SIZE-1);
        return idx;
    }

    public static LookupTableEntry getLookupTableEntry(double distanceFt) {
        return LOOKUP_TABLE[distanceToIndex(distanceFt)];
    }

    /**
     * f(x) to computes optimal time
     */

    public static double optimizeTime(double xFt, double ydFt) {
        double nu = (4.0 * (Math.pow(xFt, 2) + Math.pow(ydFt, 2)));
        double de = (Math.pow(GRAVITY, 2));
        double inside = nu/de;
        return Math.pow(inside, 0.25); // 1/4 power
    }

    /**
     * Compute best velocity and angle with limit:
     *
     * 1. call optimieTime to get optimal shoot time `t` between hood and hub
     * 2. if returned angle is > 45, decrease time until angle is <= 45
     * 3. if returned angle is < 15, increase time until angle is >= 45
     *
     * angle is directly proportional to time
     * velocity is inversely proportial to time
     *
     *  @param xFt: horizontal distance (feet)
     *  @param yiFt: shooter hood height (feet)
     *  @param yfFt: hub height (feet)
     */

    public static ShooterValues computeVelocityAndAngleWithLimit(
            double xFt,
            double yiFt,
            double yfFt,
            double minAngleDeg,
            double maxAngleDeg) {

        if (xFt <= 0) return null;
        if (minAngleDeg >= maxAngleDeg) return null;

        double ydFt = yfFt - yiFt;

        // find the optimal time using desmos formula
        double time = optimizeTime(xFt, ydFt);

        // make sure the time returned is between 0.05 and 3.00 seconds
        // to avoid 0 or huge time
        time = clamp(time, TIME_MIN, TIME_MAX);

        // delta time - how much to increase or decrease while adjusting time
        // maximum iterations of 295
        // (3.00 - 0.05)/0.01 = 295
        double dt = 0.01; // 10 ms steps
        int maxIteration = (int) ((TIME_MAX - TIME_MIN) / dt);


        for (int iter = 0; iter < maxIteration; iter++) {

            ShooterValues s = computeVelocityAndAngle(xFt, yiFt, ydFt, time);
            if (s == null) {
                return null;
            }

            //System.out.println("Sahana:search_for_better: " + s);

            // If angle is in range, we are done
            if (s.angleDeg >= minAngleDeg && s.angleDeg <= maxAngleDeg) {

                // add values from the lookup tanle before return
                LookupTableEntry lookupEntry = getLookupTableEntry(xFt);
                ShooterValues sr = new ShooterValues(s.distanceFt,
                        s.timeSec,
                        s.vxFtPerSec ,
                        s.vyFtPerSec,
                        s.velocityFtPerSec + lookupEntry.velocityOffsetFtPerSec,
                        s.angleDeg + lookupEntry.angleOffsetDeg);
                return s;
            }

            // angle too high => shot too steep => time too big => reduce time
            if (s.angleDeg > maxAngleDeg) {
                time -= dt;
            }
            // Angle too low => shot too flat => time too small => increase time
            else if (s.angleDeg < minAngleDeg) {
                time += dt;
            }

            // unrealistic time => no solution
            if (time < TIME_MIN || time > TIME_MAX) {
                break;
            }

        }
        // no result
        return null;
    }

    /**
     * Given timeSec timeSec, compute vx, vy, angle, and speed.
     *
     *  @param xFt: horizontal distance (feet)
     *  @param yiFt: shooter hood height (feet)
     *  @param yfFt: hub height (feet)
     *  @param timeSec: best timeSec from desmos
     *
     */
    private static ShooterValues computeVelocityAndAngle(
            double xFt,
            double yiFt,
            double yfFt,
            double timeSec) {

        if (timeSec <= 0) return null;

        // vx = x/timeSec
        double vx = xFt / timeSec;

        // vy from desmos
        double vy = (0.5 * GRAVITY * timeSec * timeSec + yiFt - yfFt) / (-timeSec);

        double angleDeg = Math.toDegrees(Math.atan2(vy, vx));

        // sqrt(vx*vx + vy*vy)
        double speed = Math.hypot(vx, vy);

        return new ShooterValues(xFt, timeSec, vx, vy, speed, angleDeg);
    }

    /**
     *
     * @param timeValue calculated optimal time
     * @param lo lowest time possible
     * @param hi highest time possible
     * @return optimal time within the possible time range
     */

    private static double clamp(double timeValue, double lo, double hi) {
        return Math.max(lo, Math.min(hi, timeValue));
    }

    // Quick demo
    public static void main(String[] args) {
        // Example
        for (int i=1; i<33; i++) {
            ShooterValues sol = computeVelocityAndAngleWithLimit(
                    i,
                    2.0,
                    7.0,
                    15.0,
                    45.0);
            System.out.println(sol != null ? sol : "No valid solution");
        }
    }
}
