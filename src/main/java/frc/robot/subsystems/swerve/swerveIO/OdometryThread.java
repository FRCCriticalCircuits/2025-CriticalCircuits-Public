package frc.robot.subsystems.swerve.swerveIO;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

public class OdometryThread {
    private final List<DoubleSupplier> signals = new ArrayList<>();
    private final List<Queue<Double>> queues = new ArrayList<>();

    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static OdometryThread instance = null;
    private Notifier notifier = new Notifier(this::run);

    public static OdometryThread getInstance() {
        if (instance == null) {
            instance = new OdometryThread();
        }
        return instance;
    }

    private OdometryThread() {
        notifier.setName("OdometryThread");
    }

    public void start() {
        if(!timestampQueues.isEmpty()) notifier.startPeriodic(1.0 / PhysicalConstants.ODOMETRY_FREQUENCY);
    }

    /** Registers a generic signal to be read from the thread. */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        SwerveSubsystem.odometryLock.lock();
        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            SwerveSubsystem.odometryLock.unlock();
        }
        return queue;
    }

    /** Returns a new queue that returns timestamp values for each sample. */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        SwerveSubsystem.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            SwerveSubsystem.odometryLock.unlock();
        }
        return queue;
    }

    private void run() {
        // Save new data to queues
        SwerveSubsystem.odometryLock.lock();
        try {
            // Get sample timestamp
            double timestamp = RobotController.getFPGATime() / 1e6;

            for (int i = 0; i < signals.size(); i++) {
                queues.get(i).offer(signals.get(i).getAsDouble());
            }

            for (int i = 0; i < timestampQueues.size(); i++) {
                timestampQueues.get(i).offer(timestamp);
            }
        } finally {
            SwerveSubsystem.odometryLock.unlock();
        }
    }
}
