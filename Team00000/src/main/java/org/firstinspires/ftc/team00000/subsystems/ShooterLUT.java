package org.firstinspires.ftc.team00000.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterLUT {
    /** Piecewise table (meters → RPM). Fill from calibration. */
    public static double[] D = {1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
    public static double[] RPM = {2500, 2700, 2850, 3000, 3150, 3300};

    public static double rpmForDistance(double meters) {
        if (D.length == 0 || RPM.length == 0 || D.length != RPM.length) return 0;
        if (Double.isNaN(meters)) return 0;
        if (meters <= D[0]) return RPM[0];
        for (int i=1; i<D.length; i++) {
            if (meters <= D[i]) {
                double t = (meters - D[i-1]) / (D[i] - D[i-1]);
                return RPM[i-1] + t * (RPM[i] - RPM[i-1]);
            }
        }
        return RPM[RPM.length-1];
    }
}