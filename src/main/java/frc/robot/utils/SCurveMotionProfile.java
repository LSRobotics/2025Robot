package frc.robot.utils;

public class SCurveMotionProfile {
  
  public static class State {
    public final double position;
    public final double velocity;
    public final double acceleration;

    public State(double position, double velocity, double acceleration) {
      this.position = position;
      this.velocity = velocity;
      this.acceleration = acceleration;
    }
  }

  private final double initialPos;
  private final double initialVel;
  private final double finalPos;
  private final double finalVel;
  private final double maxVel;
  private final double maxAcc;
  private final double maxJerk;

  private final double t1, t2, t3, t4, t5, t6, t7;

  private final double totalTime;

  private final double v1, v2, v3, v4, v5, v6, v7;
  private final double p1, p2, p3, p4, p5, p6, p7;

  public SCurveMotionProfile(double initialPos, double initialVel,
                             double finalPos, double finalVel,
                             double maxVel, double maxAcc, double maxJerk) {
    this.initialPos = initialPos;
    this.initialVel = initialVel;
    this.finalPos = finalPos;
    this.finalVel = finalVel;
    this.maxVel = maxVel;
    this.maxAcc = maxAcc;
    this.maxJerk = maxJerk;



    double tj = maxAcc / maxJerk;


    double ta = (maxVel / maxAcc) - tj;

    if (ta < 0) {
      ta = 0;
      double adjustedTj = Math.cbrt(maxVel / maxJerk);
      t1 = adjustedTj;
      t2 = 0;
      t3 = adjustedTj;
      t5 = adjustedTj;
      t6 = 0;
      t7 = adjustedTj;
    } else {
      t1 = tj;
      t2 = ta;
      t3 = tj;
      t5 = tj;
      t6 = ta;
      t7 = tj;
    }


    double accelDistance = 
      maxJerk * Math.pow(t1, 3)/6 + 
      maxAcc * t2 * (t1 + t2/2) + 
      maxJerk * Math.pow(t3, 3)/6 + 
      maxVel * 0;  

    double distance = Math.abs(finalPos - initialPos);

    double accelDecelDistance = 2 * (maxAcc * (t1 + t2 + t3) * (maxVel / maxAcc) / 2);

    double t4temp = (distance - accelDecelDistance) / maxVel;
    t4 = Math.max(0, t4temp);

    totalTime = t1 + t2 + t3 + t4 + t5 + t6 + t7;


    v1 = initialVel + maxJerk * t1 * t1 / 2;
    v2 = v1 + maxAcc * t2;
    v3 = v2 + maxAcc * t3 - maxJerk * t3 * t3 / 2;
    v4 = maxVel; 

    v5 = v4 - maxJerk * t5 * t5 / 2;
    v6 = v5 - maxAcc * t6;
    v7 = v6 - maxAcc * t7 + maxJerk * t7 * t7 / 2;

    p1 = initialPos + initialVel * t1 + maxJerk * Math.pow(t1, 3) / 6;
    p2 = p1 + v1 * t2 + maxAcc * t2 * t2 / 2;
    p3 = p2 + v2 * t3 + maxAcc * t3 * t3 / 2 - maxJerk * Math.pow(t3, 3) / 6;
    p4 = p3 + v3 * t4;
    p5 = p4 + v4 * t5 - maxJerk * Math.pow(t5, 3) / 6;
    p6 = p5 + v5 * t6 - maxAcc * t6 * t6 / 2;
    p7 = p6 + v6 * t7 - maxAcc * t7 * t7 / 2 + maxJerk * Math.pow(t7, 3) / 6;
  }


  public double getTotalTime() {
    return totalTime;
  }


  public State sample(double t) {
    if (t <= 0) {
      return new State(initialPos, initialVel, 0);
    } else if (t < t1) {
      double a = maxJerk * t;
      double v = initialVel + maxJerk * t * t / 2;
      double p = initialPos + initialVel * t + maxJerk * Math.pow(t, 3) / 6;
      return new State(p, v, a);

    } else if (t < t1 + t2) {
      double dt = t - t1;
      double a = maxAcc;
      double v = v1 + maxAcc * dt;
      double p = p1 + v1 * dt + maxAcc * dt * dt / 2;
      return new State(p, v, a);

    } else if (t < t1 + t2 + t3) {
      double dt = t - t1 - t2;
      double a = maxAcc - maxJerk * dt;
      double v = v2 + maxAcc * dt - maxJerk * dt * dt / 2;
      double p = p2 + v2 * dt + maxAcc * dt * dt / 2 - maxJerk * Math.pow(dt, 3) / 6;
      return new State(p, v, a);

    } else if (t < t1 + t2 + t3 + t4) {
      double dt = t - t1 - t2 - t3;
      double a = 0;
      double v = maxVel;
      double p = p3 + v * dt;
      return new State(p, v, a);

    } else if (t < t1 + t2 + t3 + t4 + t5) {
      double dt = t - t1 - t2 - t3 - t4;
      double a = -maxJerk * dt;
      double v = v4 - maxJerk * dt * dt / 2;
      double p = p4 + v4 * dt - maxJerk * Math.pow(dt, 3) / 6;
      return new State(p, v, a);

    } else if (t < t1 + t2 + t3 + t4 + t5 + t6) {
      double dt = t - t1 - t2 - t3 - t4 - t5;
      double a = -maxAcc;
      double v = v5 - maxAcc * dt;
      double p = p5 + v5 * dt - maxAcc * dt * dt / 2;
      return new State(p, v, a);

    } else if (t < totalTime) {
      double dt = t - t1 - t2 - t3 - t4 - t5 - t6;
      double a = -maxAcc + maxJerk * dt;
      double v = v6 - maxAcc * dt + maxJerk * dt * dt / 2;
      double p = p6 + v6 * dt - maxAcc * dt * dt / 2 + maxJerk * Math.pow(dt, 3) / 6;
      return new State(p, v, a);

    } else {
      return new State(finalPos, finalVel, 0);
    }
  }
}
