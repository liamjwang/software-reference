# Tuning a Velocity PID

_Based on the CTRE Software Reference Manual_

This guide uses classes from Team 1540's reusable code library, [ROOSTER](https://github.com/flamingchickens1540/ROOSTER); consult its documentation for more info.

## Step 1: Determine Maximum Velocity Experimentally

Create simple test code to drive the mechanism at a specified throttle and report the velocity (as measured by the encoders native units per 100ms) back to the SmartDashboard/Shuffleboard. For drivetrain tuning, this should be accomplished through joystick control to avoid hitting things, but for mechanisms such as flywheels simply setting the throttle through a tunable value is sufficient.

!!! note "Use Working Load When Tuning"
	Throughout the process of tuning the PID controller the motor should be under the same load as it will be used in competitions, i.e. don't try to tune velocity PIDs on a drive train while the robot is off of the ground.

### Example for Flywheel Control

```java
public class FlywheelTestRobot extends IterativeRobot {
  private ChickenTalon flywheelMaster = new ChickenTalon(0);
  private ChickenTalon flywheelSlave = new ChickenTalon(1);
  @Tunable("Throttle")
  public double throttle = 0.0;
  
  @Override
  public void robotInit() {
    AdjustableManager.getInstance().add(this);
    
    flywheelMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    flywheelSlave.set(ControlMode.Follower, flywheelMaster.getDeviceID());
  }
  
  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run(); // needed for the AdjustableManager to work
    SmartDashboard.putNumber("Velocity", flywheelMaster.getSelectedSensorVelocity());
  }
  
  @Override
  public void teleopPeriodic() {
    flywheelMaster.set(ControlMode.PercentOutput, throttle);
  }
}
```

In this case, simply set the motor output to 100% and monitor the velocity reading. Once it stabilizes, record it; this will be used to calculate F-gain.

### Example for Drivetrain Control

```java hl_lines="27 28 33 34"
public class DriveTestRobot extends IterativeRobot {
  private static final int LEFT_Y_AXIS = 1;
  private static final int RIGHT_Y_AXIS = 5;
  
  private Joystick joystick = new Joystick(0);
  private ChickenTalon lMaster = new ChickenTalon(1);
  private ChickenTalon lSlave1 = new ChickenTalon(2);
  private ChickenTalon lSlave2 = new ChickenTalon(3);
  private ChickenTalon rMaster = new ChickenTalon(4);
  private ChickenTalon rSlave1 = new ChickenTalon(5);
  private ChickenTalon rSlave2 = new ChickenTalon(6);

  @Override
  public void robotInit() {
    // setting brake mode, inversions, ramp rates, etc. ommitted for brevity
    lMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    lSlave1.set(ControlMode.Follower, lMaster.getDeviceID());
    lSlave2.set(ControlMode.Follower, lMaster.getDeviceID());

    rMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rSlave1.set(ControlMode.Follower, rMaster.getDeviceID());
    rSlave2.set(ControlMode.Follower, rMaster.getDeviceID());
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("L Vel", lMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("R Vel", rMaster.getSelectedSensorVelocity());
  }

  @Override
  public void teleopPeriodic() {
    lMaster.set(ControlMode.PercentOutput, joystick.getRawAxis(LEFT_Y_AXIS));
    rMaster.set(ControlMode.PercentOutput, joystick.getRawAxis(RIGHT_Y_AXS));
  }
}
```

In this case, simply drive the robot up to full speed using joysticks (try not to hit anything) and check the maximum speed attained. (Graphs are highly useful for this.)

## Step 2: Determine F-gain

Once the maximum output of the mechanism is established, use it to calculate your F-gain. The F-gain determines the base motor output when a specific velocity is commanded and does not use feedback from encoders. Since Talon SRXs represent throttle as a 10-bit signed integer from -1023 to 1023, an F-gain of (1023 / 500) or 2.046 would cause the Talon to output 100% when a speed of 500 is commanded. In that sense, divide 1023 by your measured maximum speed to get your F-gain.

## Step 3: Tune P, I, and D

Write a test robot which constantly applies tunable P, I, and D coefficients, as well as commanding the motor to go to a specific velocity. For flywheels, this can be set directly through the SmartDashboard; for drivetrains, this can be set by multiplying joystick values by your predetermined maximum velocity.

Tune your PID coefficients according to the Talon SRX Software Reference Manual. Start with a P-gain of about 0.1 and the I and D values set to zero.

> Double the P-gain until the system oscillates (too much) or until the system responds adequately.
>
> If the mechanism is moving to swiftly, you can add D-gain to smooth the motion. Start with 10x the P-gain.
>
> If the mechanism is not quite reaching the final target position (and P-gain cannot be increased further without hurting overall performance) begin adding I-gain. Start with 1/100th of the P-gain.

When adjusting the I-gain, it is important to also adjust the iZone coefficient which represents the size in milliseconds of the rolling window used to calculate the integral value. Try starting with a iZone value of 1000 (1 second). When increasing the iZone coefficient, the I coefficient should be decreased.

### Tuning Tips and Tricks

If (as in the case of a flywheel) you only care about keeping the mechanism at a certain speed when it is moving, and don't care about how fast it can stop, either prevent the motor from running in reverse (using the `configPeakOutputForward()` or `configPeakOutputReverse()` methods of `ChickenTalon`) or add code to disable PID control when stopping.

If you find the mechanism consuming too much power or having trouble getting up to speed due to brownouts, add a small amount of closed-loop ramping using the `configClosedLoopRamp()` method. A good value for drivetrains is between 0.1 and 0.3 seconds. (Ramping is measured in seconds for the motor to ramp up from 0% to 100%.)

Use the RoboRIO web dashboard to confirm that the PIDF coefficients are being set and that the CANTalon is receiving feedback from the correct encoder. 

!!! bug "Not Our Fault"
	Ramp rates are **not** displayed on the web dashboard; there is a field called "ramp rate", but setting it has no effect and it will always read 0. (See the CTRE Software Reference Manual, section 6.3.)

## Common Problems

| Problem                                  | Solution                                 |
| ---------------------------------------- | ---------------------------------------- |
| Mechanism is oscillating.                | Increase the D coefficient. If this worsens the oscillation, reduce the P, I, or D coefficients one at a time to isolate the cause of the oscillation. |
| Mechanism is too slow to respond to a command. | Lower the amount of closed loop ramping using the `configClosedLoopRamp()` method. If that fails (ramping is already at 0), increase the F​ coefficient. |
| Mechanism velocity stabilizes at a value far from the goal. | Adjust the F coefficient (with the other coefficients set to zero) until the actual speed is as close as possible to the goal. |
| When P, I, or D is nonzero, the mechanism quickly accelerates to maximum output regardless of commanded velocity. | Flip the sensor phase using `setSensorPhase()`. |
