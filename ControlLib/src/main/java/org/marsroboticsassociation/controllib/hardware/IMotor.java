package org.marsroboticsassociation.controllib.hardware;

public interface IMotor {
    String getName();

    int getPosition(); // encoder ticks

    double getVelocity(); // ticks per second

    void setPower(double power);

    double getHubVoltage(); // volts, live read

    void setVelocity(double tps);

    void setVelocityPIDFCoefficients(double p, double i, double d, double f);
}
