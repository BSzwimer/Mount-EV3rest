package ca.mcgill.ecse211.mountev3rest.navigation;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.mountev3rest.navigation.Odometer;
import ca.mcgill.ecse211.mountev3rest.navigation.OdometerException;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta)
 */
public class Display implements Runnable {

  private Odometer odo;
  private TextLCD lcd;
  private SampleProvider gyro;
  private double[] position;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * This is the class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd) throws OdometerException {
    odo = Odometer.getOdometer();
    this.lcd = lcd;
  }
  
  /**
   * Overloaded constructor to include gyro sensor.
   */
  public Display(TextLCD lcd, EV3GyroSensor gyro) throws OdometerException {
    odo = Odometer.getOdometer();
    this.lcd = lcd;
    this.gyro = gyro.getAngleMode();
  }

  /**
   * This is the overloaded class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd, long timeout) throws OdometerException {
    odo = Odometer.getOdometer();
    this.timeout = timeout;
    this.lcd = lcd;
  }

  public void run() {
    
    float[] sample = new float[1];
    if (gyro != null) {
      sample = new float[gyro.sampleSize()];
    }
    
    lcd.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = odo.getXYT();
      
      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      
      if (gyro != null) {
        gyro.fetchSample(sample, 0);
        lcd.drawString("G: " + numberFormat.format(sample[0]), 0, 4);
      }
      
      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }

}
