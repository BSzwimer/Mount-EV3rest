package ca.mcgill.ecse211.mountev3rest.navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

/**
 * Uses the measurements of the wheel radius, track length and tacho meter readings to provide a
 * real time estimation of the cart in a 2 dimensional plane.
 * 
 * @author angelortiz
 *
 */
public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private int prevLeftMotorTachoCount = 0;
  private int prevRightMotorTachoCount = 0;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private SampleProvider gyroSensor;
  private float[] gyroSample;
  private float prevGyroSample;

  private final double WHEEL_RAD;
  private final double MOTOR_OFFSET;


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerException
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      EV3GyroSensor gyroSensor, final double WHEEL_RAD, final double MOTOR_OFFSET)
      throws OdometerException {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.gyroSensor = gyroSensor.getAngleMode();
    this.gyroSample = new float[this.gyroSensor.sampleSize()];
    this.prevGyroSample = 0;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.WHEEL_RAD = WHEEL_RAD;
    this.MOTOR_OFFSET = MOTOR_OFFSET;
  }

  /**
   * {@code Odometer} factory method. Creates and instance of the {@code Odometer} class if it has
   * not been instantiated yet, otherwise it returns the existing instance.
   * 
   * @param leftMotor Left motor of the robot.
   * @param rightMotor Right motor of the robot.
   * @param track Track measurement of the robot.
   * @param wheelRadius Wheel radius measurement of the robot.
   * 
   * @return New or existing {@code Odometer} object.
   * @throws OdometerException If there is a problem while instantiating the new {@code Odometer}
   *         object.
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, EV3GyroSensor gyroSensor, final double wheelRadius,
      double motorOffset) throws OdometerException {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, gyroSensor, wheelRadius, motorOffset);
      return odo;
    }
  }

  /**
   * Returns the existing instance of the singleton {@code Odometer} class.
   * 
   * @return Existing instance of the {@code Odometer} class.
   * @throws OdometerException If the {@code Odometer} has not been instantiated.
   */
  public synchronized static Odometer getOdometer() throws OdometerException {

    if (odo == null) {
      throw new OdometerException("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * Uses the wheel radius, track length, and tacho meter measurements from the motors to update the
   * X and Y values as well the angle Theta of the cart's current position.
   */
  @Override
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      double[] position = odo.getXYT();

      leftMotorTachoCount = (int) (leftMotor.getTachoCount() / MOTOR_OFFSET);
      rightMotorTachoCount = rightMotor.getTachoCount();

      // Calculate new robot position based on tachometer counts
      double distL = Math.PI * WHEEL_RAD * (leftMotorTachoCount - prevLeftMotorTachoCount) / 180;
      double distR = Math.PI * WHEEL_RAD * (rightMotorTachoCount - prevRightMotorTachoCount) / 180;
      double deltaD = 0.5 * (distL + distR);
      gyroSensor.fetchSample(gyroSample, 0);
      double deltaT = -1 * (gyroSample[0] - prevGyroSample);

      // Get the current odometer values
      double deltaX = deltaD * Math.sin(Math.toRadians(position[2] + deltaT));
      double deltaY = deltaD * Math.cos(Math.toRadians(position[2] + deltaT));

      // Update odometer values with new calculated values
      odo.update(deltaX, deltaY, deltaT);

      // Set current values to be the old values
      prevLeftMotorTachoCount = leftMotorTachoCount;
      prevRightMotorTachoCount = rightMotorTachoCount;
      prevGyroSample = gyroSample[0];

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

  /**
   * Read the reading of the gyro sensor.
   * 
   * @return Gyro sensor reading in degrees.
   */
  public float pollGyro() {
    float[] sample = new float[gyroSensor.sampleSize()];
    gyroSensor.fetchSample(sample, 0);
    return sample[0];
  }

}
