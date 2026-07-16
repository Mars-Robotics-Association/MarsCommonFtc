package org.marsroboticsassociation.controllib.mechanism;

/**
 * PIDF position controller for a DC-motor driven mechanism, the companion to {@link
 * MotorMechanismEkf}: the filter estimates where the mechanism is, this drives it where you want it
 * to go. Both take the same {@link MechanismModel}.
 *
 * <p>Each loop it does four things:
 *
 * <ol>
 *   <li><b>Profiles the target</b> through a {@link ModelAwareSetpointProfile} (a {@link
 *       ModelAwareRuckigProfiler} unless the caller supplies one), so the setpoint it chases is a
 *       planned, jerk-limited trajectory that lands at rest exactly on the target.
 *   <li><b>Feeds forward</b> the voltage the model says the motion needs (the F in PIDF), including
 *       the back-EMF term.
 *   <li><b>Corrects with PID</b> on the gap between the profile and the filter's estimate.
 *   <li><b>Clamps</b> the result to the bus voltage.
 * </ol>
 *
 * <p>The measured <b>bus voltage</b> is passed in every loop, not fixed at construction, because an
 * FTC battery is only nominally 12 V: a fresh pack is a bit over 14 V and it sags under load and
 * drains over a match. Every limit keyed off voltage (the clamp and the back-EMF ceilings) tracks
 * that live value. The output is a voltage; convert it to a motor power with the same reading
 * ({@code power = volts / busVolts}) so sag does not change the behavior, and feed that applied
 * voltage back into the filter's predict step.
 *
 * <p><b>Back-EMF ceilings.</b> The profile owns them: before every replan it evaluates the shared
 * model at its own state and caps its velocity, acceleration, and deceleration limits to what the
 * motor can actually sustain there (see {@link ModelAwareRuckigProfiler} for how gravity and
 * braking are charged). The controller's only per-loop contribution is telling the profile how
 * much voltage this loop has — the bus voltage minus the feedback margin.
 */
public class MotorMechanismController {

    private final MechanismModel model;

    // PID feedback gains, in volts per unit of error.
    private final double kP;
    private final double kI;
    private final double kD;

    private final double feedbackVoltageMargin;

    private final ModelAwareSetpointProfile profile;
    private double halfBacklash = 0.0;
    private double backlashTaperVolts = 0.0;
    private double restComplianceRadPerVolt = 0.0;
    private double integral = 0.0;
    private double lastVoltage = 0.0;

    /**
     * Build with the default profiler, a {@link ModelAwareRuckigProfiler} under symmetric
     * accel/decel caps.
     *
     * @param model the shared mechanism model (arm or lift)
     * @param kP proportional gain, volts per unit of error
     * @param kI integral gain, volts per (unit-second)
     * @param kD derivative gain, volts per (unit/sec) of velocity error
     * @param maxVelocity velocity limit
     * @param maxAcceleration acceleration limit (the mechanical cap for both speeding up and
     *     braking; back-EMF throttles acceleration below it at speed)
     * @param maxJerk jerk limit
     * @param feedbackVoltageMargin volts held back from the profile's feedforward so the PID has
     *     room to correct (e.g. 1.5); the bus voltage comes in per loop
     * @param initialPosition the mechanism's position right now
     */
    public MotorMechanismController(
            MechanismModel model,
            double kP,
            double kI,
            double kD,
            double maxVelocity,
            double maxAcceleration,
            double maxJerk,
            double feedbackVoltageMargin,
            double initialPosition) {
        this(
                model,
                kP,
                kI,
                kD,
                maxVelocity,
                maxAcceleration,
                maxAcceleration,
                maxJerk,
                feedbackVoltageMargin,
                initialPosition);
    }

    /**
     * As the symmetric constructor, but with independent mechanical caps for speeding up and
     * braking. Both are motion-frame limits ({@link SetpointProfile}): {@code maxAcceleration}
     * bounds gaining speed, {@code maxDeceleration} bounds braking, regardless of direction. Pass
     * {@code maxAcceleration > maxDeceleration} to let a mechanism launch harder than it stops
     * (e.g. a flexible arm whose arrival swing limits braking, not starting).
     *
     * @param maxAcceleration mechanical cap on acceleration that increases speed
     * @param maxDeceleration mechanical cap on braking
     */
    public MotorMechanismController(
            MechanismModel model,
            double kP,
            double kI,
            double kD,
            double maxVelocity,
            double maxAcceleration,
            double maxDeceleration,
            double maxJerk,
            double feedbackVoltageMargin,
            double initialPosition) {
        this(
                model,
                kP,
                kI,
                kD,
                feedbackVoltageMargin,
                new ModelAwareRuckigProfiler(
                        model,
                        maxVelocity,
                        maxAcceleration,
                        maxDeceleration,
                        maxJerk,
                        initialPosition));
    }

    /**
     * Construct with a caller-supplied setpoint profiler, e.g. a {@link ModelAwareRuckigProfiler}
     * built with non-default limits. The profile computes its own back-EMF ceilings at plan time
     * from the shared model; the controller only forwards the available voltage each loop.
     *
     * @param profile the setpoint profiler, already configured with its limits and initial
     *     position
     */
    public MotorMechanismController(
            MechanismModel model,
            double kP,
            double kI,
            double kD,
            double feedbackVoltageMargin,
            ModelAwareSetpointProfile profile) {
        this.model = model;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.feedbackVoltageMargin = feedbackVoltageMargin;
        this.profile = profile;
    }

    /**
     * Enable rest-only backlash compensation. A motor-encoder controller can only servo the motor
     * side of the gearbox; the load settles a half-backlash away from it, hanging on the
     * gravity-loaded tooth face. Biasing the target a half-backlash <em>against</em> gravity puts
     * the load, not the motor, on the stated target. Only the endpoint moves — the profile plans
     * to the biased target from the start, so the motion is unchanged and there is no correction
     * hop at arrival.
     *
     * <p>The bias is scaled by {@code gravityVoltage(target) / taperVolts}, clamped to ±1, so it
     * carries gravity's sign (an arm past vertical rests on the other face) and fades to zero near
     * a crest, where gravity is too weak to pin the resting face and a full bias would be a coin
     * flip that can double the error.
     *
     * <p>The biased target sits up to a half-backlash past the stated one: keep stated targets at
     * least that far inside any hard stop. Pass {@code backlashRad} of 0 to disable.
     *
     * @param backlash total lash at the output, in the mechanism's position units
     * @param taperVolts gravity hold-voltage below which the bias tapers toward zero (must be
     *     positive; e.g. the gravity voltage a few degrees off the crest)
     */
    public void setBacklashCompensation(double backlash, double taperVolts) {
        setBacklashCompensation(backlash, taperVolts, 0.0);
    }

    /**
     * As {@link #setBacklashCompensation(double, double)}, plus a <b>rest compliance</b> term for
     * elastic droop: gear-tooth contact penetration and structural (arm-tube) sag both let the
     * load rest {@code compliance · gravityVoltage(target)} beyond the half-lash, proportionally
     * to gravity. Unlike the lash term this needs no crest taper — it vanishes with gravity on its
     * own. On a real mechanism, measure the rest droop at two angles: the constant part is the
     * half-lash, the gravity-proportional slope is this compliance.
     *
     * @param backlash total lash at the output, in the mechanism's position units
     * @param taperVolts gravity hold-voltage below which the lash bias tapers toward zero
     * @param complianceRadPerVolt elastic rest droop per volt of gravity hold-voltage (0 for a
     *     stiff mechanism)
     */
    public void setBacklashCompensation(
            double backlash, double taperVolts, double complianceRadPerVolt) {
        if (backlash < 0) {
            throw new IllegalArgumentException("backlash must be >= 0, got " + backlash);
        }
        if (backlash > 0 && taperVolts <= 0) {
            throw new IllegalArgumentException("taperVolts must be > 0, got " + taperVolts);
        }
        if (complianceRadPerVolt < 0) {
            throw new IllegalArgumentException(
                    "complianceRadPerVolt must be >= 0, got " + complianceRadPerVolt);
        }
        this.halfBacklash = backlash / 2.0;
        this.backlashTaperVolts = taperVolts;
        this.restComplianceRadPerVolt = complianceRadPerVolt;
    }

    /** Restart the profile from a known position at rest and clear the integrator. */
    public void reset(double position) {
        profile.reset(position);
        integral = 0.0;
        lastVoltage = 0.0;
    }

    /**
     * Compute the commanded motor voltage for this loop.
     *
     * @param targetPosition where you want the mechanism to end up
     * @param measuredPosition the filter's position estimate
     * @param measuredVelocity the filter's velocity estimate
     * @param busVoltage the measured battery/bus voltage this loop
     * @param dt seconds since the previous call
     * @return commanded voltage, clamped to +/- busVoltage
     */
    public double calculate(
            double targetPosition,
            double measuredPosition,
            double measuredVelocity,
            double busVoltage,
            double dt) {
        // Rest-only backlash compensation (see setBacklashCompensation): the profile chases the
        // biased endpoint so the load comes to rest on the stated one.
        targetPosition = compensatedTarget(targetPosition);

        // Voltage available to the feedforward, leaving a margin for the PID to correct with. The
        // profile computes its own back-EMF ceilings from the shared model at plan time; it only
        // needs to know how much voltage this loop has.
        double availableVoltage = Math.max(0.0, busVoltage - feedbackVoltageMargin);
        profile.setAvailableVoltage(availableVoltage);

        profile.update(targetPosition, dt);
        double setpointPosition = profile.getPosition();
        double setpointVelocity = profile.getVelocity();
        double setpointAcceleration = profile.getAcceleration();

        // F: feedforward from the model (back-EMF aware).
        double feedforward =
                model.feedforwardVoltage(setpointPosition, setpointVelocity, setpointAcceleration);

        // PID feedback on the profile-versus-estimate gap.
        double positionError = setpointPosition - measuredPosition;
        double velocityError = setpointVelocity - measuredVelocity;
        double tentativeIntegral = integral + positionError * dt;
        double feedback = kP * positionError + kI * tentativeIntegral + kD * velocityError;

        double voltage = feedforward + feedback;
        double clamped = clamp(voltage, -busVoltage, busVoltage);
        // Anti-windup: only keep the new integral if the command was not saturated.
        if (clamped == voltage) {
            integral = tentativeIntegral;
        }

        lastVoltage = clamped;
        return clamped;
    }

    /** The voltage returned by the most recent {@link #calculate}. */
    public double getLastVoltage() {
        return lastVoltage;
    }

    public double getSetpointPosition() {
        return profile.getPosition();
    }

    public double getSetpointVelocity() {
        return profile.getVelocity();
    }

    public double getSetpointAcceleration() {
        return profile.getAcceleration();
    }

    /**
     * The target the profile actually chases for a stated target: the stated position plus the
     * rest-only backlash bias (identity when compensation is disabled). Exposed so callers can
     * score the profile against its true endpoint, e.g. in telemetry or arrival checks.
     */
    public double compensatedTarget(double targetPosition) {
        return targetPosition + backlashBias(targetPosition);
    }

    /**
     * The rest bias for a stated target: a half-backlash signed by which tooth face gravity loads
     * there (tapered where the gravity hold-voltage is below {@code backlashTaperVolts}), plus the
     * gravity-proportional elastic droop.
     */
    private double backlashBias(double target) {
        if (halfBacklash == 0.0 && restComplianceRadPerVolt == 0.0) {
            return 0.0;
        }
        double gravity = model.gravityVoltage(target);
        double bias = restComplianceRadPerVolt * gravity;
        if (halfBacklash > 0.0) {
            bias += halfBacklash * clamp(gravity / backlashTaperVolts, -1.0, 1.0);
        }
        return bias;
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }
}
