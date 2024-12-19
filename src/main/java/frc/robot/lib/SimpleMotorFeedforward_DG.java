package frc.robot.lib;

/** Add your docs here. */
public class SimpleMotorFeedforward_DG {
  /** The static gain, in volts. */
  private final double m_ks;

  /** The velocity gain, in V/(units/s). */
  private final double m_kv;

  /** The acceleration gain, in V/(units/s²). */
  private final double m_ka;

  public SimpleMotorFeedforward_DG(double ks, double kv, double ka) {
    this.m_ks = ks;
    this.m_kv = kv;
    this.m_ka = ka;
  }

  public double calculate(double _velocity, double _acceleration) {
    return m_ks * Math.signum(_velocity) + m_kv * _velocity + m_ka * _acceleration;
  }
}
