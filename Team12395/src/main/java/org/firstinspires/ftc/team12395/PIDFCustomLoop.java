package org.firstinspires.ftc.team12395;

/**
 * The equation we will use is:
 * u(t) = kP * e(t) + kI * int(0,t)[e(t')dt'] + kD * e'(t) + kF
 * where e(t) = r(t) - y(t) and r(t) is the setpoint and y(t) is the
 * measured value. If we consider e(t) the positional error, then
 * int(0,t)[e(t')dt'] is the total error and e'(t) is the velocity error.
 */
public class PIDFCustomLoop {

    private double kP, kI, kD, kF;
    private double setPoint;
    private double measuredValue;
    private double minIntegral, maxIntegral;
    private double lastReturn;

    private double errorVal_p;
    private double lastVel;

    private double totalError;
    private double prevErrorVal;

    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;

    private double lastTimeStamp;
    private double period;

    /**
     * The base constructor for the PIDF controller
     */
    public PIDFCustomLoop(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 0, 0);
    }

    /**
     * This is the full constructor for the PIDF controller. Our PIDF controller
     * includes a feed-forward value which is useful for fighting friction and gravity.
     * Our errorVal represents the return of e(t) and prevErrorVal is the previous error.
     *
     * @param sp The setpoint of the pid control loop.
     * @param pv The measured value of he pid control loop. We want sp = pv, or to the degree
     *           such that sp - pv, or e(t) < tolerance.
     */
    public PIDFCustomLoop(double kp, double ki, double kd, double kf, double sp, double pv) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;

        setPoint = sp;
        measuredValue = pv;

        minIntegral = -1.0;
        maxIntegral = 1.0;

        lastTimeStamp = 0;
        period = 0;

        errorVal_p = setPoint - measuredValue;
        reset();
    }

    public void reset() {
        totalError = 0;
        prevErrorVal = 0;
        lastTimeStamp = 0;
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = positionTolerance;
        errorTolerance_v = velocityTolerance;
    }

    /**
     * Returns the current setpoint of the PIDFController.
     *
     * @return The current setpoint.
     */
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Sets the setpoint for the PIDFController
     *
     * @param sp The desired setpoint.
     */
    public void setSetPoint(double sp) {
        setPoint = sp;
        errorVal_p = setPoint - measuredValue;
        lastVel = (errorVal_p - prevErrorVal) / period;
    }

    /**
     * Returns true if the error is within the percentage of the total input range, determined by
     * {@link #setTolerance}.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetPoint() {
        return Math.abs(errorVal_p) < errorTolerance_p
                && Math.abs(lastVel) < errorTolerance_v;
    }

    /**
     * @return the PIDF coefficients
     */
    public double[] getCoefficients() {
        return new double[]{kP, kI, kD, kF};
    }

    /**
     * @return the positional error e(t)
     */
    public double getPositionError() {
        return errorVal_p;
    }

    /**
     * @return the tolerances of the controller
     */
    public double[] getTolerance() {
        return new double[]{errorTolerance_p, errorTolerance_v};
    }

    /**
     * @return the velocity error e'(t)
     */
    public double getVelocityError() {
        return lastVel;
    }



    public double calculate(double pv, double sp, double vel) {
        // set the setpoint to the provided value
        setSetPoint(sp);
        return calculate(pv, vel);
    }

    /**
     * Calculates the control value, u(t).
     *
     * @param pv The current measurement of the process variable.
     * @return the value produced by u(t).
     */
    public double calculate(double pv, double vel) {
        prevErrorVal = errorVal_p;

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        if (period <= 1e-6){
            return lastReturn;
        }

        errorVal_p = setPoint - pv;

        double velocity;
        if (!Double.isNaN(vel) && Double.isFinite(vel)){
            velocity = vel;
            lastVel = velocity;
        } else {
            velocity = lastVel;
        }

        /*
        if total error is the integral from 0 to t of e(t')dt', and
        e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
        totalError += period * (setPoint - measuredValue);
        totalError = totalError < minIntegral ? minIntegral : Math.min(maxIntegral, totalError);

        // returns u(t)
        lastReturn = kP * errorVal_p + kI * totalError - kD * velocity + kF * setPoint;
        return lastReturn;
    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        minIntegral = integralMin;
        maxIntegral = integralMax;
    }

    public void clearTotalError() {
        totalError = 0;
    }

    public void setP(double kp) {
        kP = kp;
    }

    public void setI(double ki) {
        kI = ki;
    }

    public void setD(double kd) {
        kD = kd;
    }

    public void setF(double kf) {
        kF = kf;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }

    public double getPeriod() {
        return period;
    }

}
