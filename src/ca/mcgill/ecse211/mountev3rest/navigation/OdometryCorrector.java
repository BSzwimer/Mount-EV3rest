package ca.mcgill.ecse211.mountev3rest.navigation;

import ca.mcgill.ecse211.mountev3rest.sensor.LightPoller;
import ca.mcgill.ecse211.mountev3rest.sensor.PollerException;

/**
 * Uses line detection on the left and right light sensors of the robot to provide periodic
 * corrections to the odometer location estimations.
 * <p>
 * Once instantiated and running, the {@code OdometryCorrector} class runs as a separate thread on
 * the background silently providing corrections to the odometer when lines are detected.
 * 
 * @author angelortiz
 *
 */
public class OdometryCorrector extends Thread {

  // Attributes
  Odometer odometer;
  LightPoller lightPoller;

  /**
   * Creates an {@code OdometryCorrector} that is to be run on its own thread.
   * @throws PollerException 
   */
  public OdometryCorrector() throws PollerException {
    lightPoller = LightPoller.getLightPoller();
  }

  /**
   * Checks for line detections on the left and right light sensors as indicated by the
   * {@code LightPoller} and corrects the values of the robot's odometer.
   * 
   * @see LightPoller
   * @see Odometer
   */
  @Override
  public void run() {
    // TODO
  }

}
