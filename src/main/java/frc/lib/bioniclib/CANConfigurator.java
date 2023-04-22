package frc.lib.bioniclib;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Vector;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Supports REV and CTRE v5 devices
 * 
 * @param T Type of the error, either REVLibError or ErrorCode
 */
public class CANConfigurator<T> {

  private final int kMaxInitAttempts = 5;
  private final double kConfigAttemptDelaySeconds = 0.04;
  private final String m_systemName;
  private final Object m_validErrorCode;
  private final List<Supplier<T>> m_configActionList = new ArrayList<>();
  private final BiConsumer<Integer, String> m_dashboardOutput;
  private List<Supplier<T>> m_retryConfigActions = new Vector<>();
  private int m_attempt = 1;
  private boolean m_retrying = false;
  public final Consumer<Supplier<T>> actionConsumer = m_configActionList::add;

  /**
   * 
   * @param systemName     The instance's unique entry name for NetworkTables
   * @param controllerType The class matching the instance's generic type
   */
  public CANConfigurator(String systemName, Class<T> controllerType) {
    m_systemName = systemName;
    if (controllerType.equals(ErrorCode.class))
      m_validErrorCode = ErrorCode.OK;
    else if (controllerType.equals(REVLibError.class))
      m_validErrorCode = REVLibError.kOk;
    else
      throw new RuntimeException("Invalid controller type class!");
    m_dashboardOutput = (k, v) -> {
      String key = "Config Errors/" + m_systemName + " [Attempt " + m_attempt + "]";
      SmartDashboard.putString(k == null ? key : key.concat(" FAILED STEP " + (k + 1)), v);
    };
  }

  public void forceConfig() {
    List<Supplier<T>> configSteps = m_retrying ? m_retryConfigActions : m_configActionList;
    Map<Integer, Supplier<T>> erroneousAttempts = configSteps.stream()
        .filter(x -> x != null && !x.get().equals(m_validErrorCode))
        .collect(Collectors.toMap(k -> m_configActionList.indexOf(k), v -> v));
    if (erroneousAttempts.isEmpty()) {
      m_dashboardOutput.accept(null, "Success");
      return;
    } else if (m_attempt > kMaxInitAttempts) {
      m_dashboardOutput.accept(null, "Max Attempts Exceeded!");
      return;
    }
    m_retrying = true;
    m_retryConfigActions = new Vector<>() {
      {
        setSize(Collections.max(erroneousAttempts.keySet()) + 1);
      }
    };
    erroneousAttempts.forEach((k, v) -> {
      m_dashboardOutput.accept(k, v.get().toString()); // Report Errors //FIXME do not call get() again
      m_retryConfigActions.set(k, v); // Create updated list w/ failed actions
    });
    m_attempt++;
    Timer.delay(kConfigAttemptDelaySeconds);
    forceConfig();
  }

  public static void configDefaultFramePeriods(CANConfigurator<REVLibError> configurator, CANSparkMax spark,
      boolean isFollower) {
    configurator.actionConsumer
        .accept(() -> spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, isFollower ? 100 : 10));
    configurator.actionConsumer.accept(() -> spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535));
    configurator.actionConsumer.accept(() -> spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535));
    configurator.actionConsumer.accept(() -> spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535));
    configurator.actionConsumer.accept(() -> spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535));
  }

  public static void configDefaultFramePeriods(CANConfigurator<ErrorCode> configurator, TalonFX talon,
      boolean isFollower) {
  }
}
