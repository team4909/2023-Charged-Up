package frc.lib.bioniclib;

import java.util.ArrayList;
import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkManager {

  private final int kMaxInitAttempts = 5;
  private ArrayList<REVLibError> m_statusList = new ArrayList<>();
  private String m_systemName;
  private int m_attempt = 1;
  private Runnable m_config;
  public Consumer<REVLibError> statusTracker = m_statusList::add;

  public SparkManager(String systemName) {
    m_systemName = systemName;
  }

  private boolean checkErrors() {
    ArrayList<REVLibError> errors = new ArrayList<>();
    Consumer<String> outputMessage = msg -> {
      SmartDashboard.putString("Config Errors/" + m_systemName + " [Attempt " + m_attempt + "]", msg);
    };
    m_statusList.forEach((e) -> {
      if (!e.equals(REVLibError.kOk))
        errors.add(e);
    });
    if (errors.size() != 0) {
      if (m_attempt < kMaxInitAttempts) {
        outputMessage.accept(errors.toString());
        return true;
      } else {
        outputMessage.accept("Failed!");
        return false;
      }
    } else {
      outputMessage.accept("No Errors");
      return false;
    }
  }

  public void forceConfig() {
    m_config.run();
    if (checkErrors()) {
      m_attempt++;
      forceConfig();
    }
  }

  public void setConfigRunnable(Runnable config) {
    m_config = config;
  }

  public void configDefaultFramePeriods(CANSparkMax spark, boolean isFollower) {
    statusTracker.accept(spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, isFollower ? 100 : 10));
    statusTracker.accept(spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535));
    statusTracker.accept(spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535));
    statusTracker.accept(spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535));
    statusTracker.accept(spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535));
  }
}
