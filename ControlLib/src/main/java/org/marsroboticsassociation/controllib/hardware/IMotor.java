package org.marsroboticsassociation.controllib.hardware;

public interface IMotor {
    double getVelocity();      // ticks per second
    void setPower(double power);
    double getHubVoltage();    // volts, live read
}
