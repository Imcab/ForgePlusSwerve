package lib.ForgePlus.NetworkTableUtils;

import java.util.Arrays;
import java.util.EnumSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A generic NetworkTables listener that listens for value updates of a given type.
 * 
 * @param <T> The type of data being monitored.
 */
public class NTListener<T> {
    private final String tableName;
    private final String key;
    private final Class<T> type;
    private T lastValue;
    private boolean changed = false;

    private NTListener(String tableName, String key, Class<T> type, T defaultValue) {
        this.tableName = tableName;
        this.key = key;
        this.type = type;
        this.lastValue = defaultValue;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable(tableName);
        NetworkTableEntry entry = table.getEntry(key);

        inst.addListener(entry, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event-> {
            T newValue = getValue();
            if (!areEqual(lastValue, newValue)) {
                lastValue = newValue;
                changed = true;
            }
        });

    }
    //Factory methods for different data types

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Double> ofDouble(String tableName, String key) {
        return new NTListener<>(tableName, key, Double.class, 0.0);
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Boolean> ofBoolean(String tableName, String key) {
        return new NTListener<>(tableName, key, Boolean.class, false);
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<String> ofString(String tableName, String key) {
        return new NTListener<>(tableName, key, String.class, "");
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<double[]> ofDoubleArray(String tableName, String key) {
        return new NTListener<>(tableName, key, double[].class, new double[]{});
    }

    /**
     * Creates a listener of the given data
    */
    public static NTListener<Pose2d> ofPose2d(String tableName, String key) {
        return new NTListener<>(tableName, key, Pose2d.class, new Pose2d());
    }

    /**
     * Creates a listener of the given data
    */
    public static NTListener<Pose3d> ofPose3d(String tableName, String key) {
        return new NTListener<>(tableName, key, Pose3d.class, new Pose3d());
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Translation2d> ofTranslation2d(String tableName, String key) {
        return new NTListener<>(tableName, key, Translation2d.class, new Translation2d());
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Translation3d> ofTranslation3d(String tableName, String key) {
        return new NTListener<>(tableName, key, Translation3d.class, new Translation3d());
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Pose2d[]> ofPose2dArray(String tableName, String key) {
        return new NTListener<>(tableName, key, Pose2d[].class, new Pose2d[]{});
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Pose3d[]> ofPose3dArray(String tableName, String key) {
        return new NTListener<Pose3d[]>(tableName, key, Pose3d[].class, new Pose3d[]{});
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Translation2d[]> ofTranslation2dArray(String tableName, String key) {
        return new NTListener<>(tableName, key, Translation2d[].class, new Translation2d[]{});
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Translation3d[]> ofTranslation3dArray(String tableName, String key) {
        return new NTListener<>(tableName, key, Translation3d[].class, new Translation3d[]{});
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Rotation2d> ofRotation2d(String tableName, String key) {
        return new NTListener<>(tableName, key, Rotation2d.class, new Rotation2d());
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Rotation3d> ofRotation3d(String tableName, String key) {
        return new NTListener<>(tableName, key, Rotation3d.class, new Rotation3d());
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<SwerveModuleState> ofSwerveModuleState(String tableName, String key) {
        return new NTListener<>(tableName, key, SwerveModuleState.class, new SwerveModuleState());
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<SwerveModulePosition> ofSwerveModulePosition(String tableName, String key) {
        return new NTListener<>(tableName, key, SwerveModulePosition.class, new SwerveModulePosition());
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<ChassisSpeeds> ofChassisSpeeds(String tableName, String key) {
        return new NTListener<>(tableName, key, ChassisSpeeds.class, new ChassisSpeeds());
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Color> ofColor(String tableName, String key) {
        return new NTListener<>(tableName, key, Color.class, new Color(0, 0, 0));
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Transform2d> ofTransform2d(String tableName, String key) {
        return new NTListener<>(tableName, key, Transform2d.class, new Transform2d());
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Transform3d> ofTransform3d(String tableName, String key) {
        return new NTListener<>(tableName, key, Transform3d.class, new Transform3d());
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Rotation2d[]> ofRotation2dArray(String tableName, String key) {
        return new NTListener<>(tableName, key, Rotation2d[].class, new Rotation2d[]{});
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Rotation3d[]> ofRotation3dArray(String tableName, String key) {
        return new NTListener<>(tableName, key, Rotation3d[].class, new Rotation3d[]{});
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<SwerveModuleState[]> ofSwerveModuleStateArray(String tableName, String key) {
        return new NTListener<>(tableName, key, SwerveModuleState[].class, new SwerveModuleState[]{});
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<SwerveModulePosition[]> ofSwerveModulePositionArray(String tableName, String key) {
        return new NTListener<>(tableName, key, SwerveModulePosition[].class, new SwerveModulePosition[]{});
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<ChassisSpeeds[]> ofChassisSpeedsArray(String tableName, String key) {
        return new NTListener<>(tableName, key, ChassisSpeeds[].class, new ChassisSpeeds[]{});
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Transform2d[]> ofTransform2dArray(String tableName, String key) {
        return new NTListener<>(tableName, key, Transform2d[].class, new Transform2d[]{});
    }

    /**
     * Creates a listener of the given data
     */
    public static NTListener<Transform3d[]> ofTransform3dArray(String tableName, String key) {
        return new NTListener<>(tableName, key, Transform3d[].class, new Transform3d[]{});
    }

    /**
     * Checks if the value has changed since the last retrieval.
     * 
     * @return {@code true} if the value has changed, otherwise {@code false}.
     */
    public boolean hasChanged() {
        if (changed) {
            changed = false;
            return true;
        }
        return false;
    }

    private synchronized boolean areEqual(T a, T b) {
    if (a == b) {
        return true;
    }
    if (a == null || b == null) {
        return false;
    }
    if (a.getClass().isArray() && b.getClass().isArray()) {
        //Handle different array types
            if (a instanceof double[] && b instanceof double[]) {
                return Arrays.equals((double[]) a, (double[]) b);
            }
            if (a instanceof Pose2d[] && b instanceof Pose2d[]) {
                return Arrays.equals((Pose2d[]) a, (Pose2d[]) b);
            }
            if (a instanceof Pose3d[] && b instanceof Pose3d[]) {
                return Arrays.equals((Pose3d[]) a, (Pose3d[]) b);
            }
            if (a instanceof Translation2d[] && b instanceof Translation2d[]) {
                return Arrays.equals((Translation2d[]) a, (Translation2d[]) b);
            }
            if (a instanceof Translation3d[] && b instanceof Translation3d[]) {
                return Arrays.equals((Translation3d[]) a, (Translation3d[]) b);
            }
            if (a instanceof Rotation2d[] && b instanceof Rotation2d[]) {
                return Arrays.equals((Rotation2d[]) a, (Rotation2d[]) b);
            }
            if (a instanceof Rotation3d[] && b instanceof Rotation3d[]) {
                return Arrays.equals((Rotation3d[]) a, (Rotation3d[]) b);
            }
            if (a instanceof SwerveModuleState[] && b instanceof SwerveModuleState[]) {
                return Arrays.equals((SwerveModuleState[]) a, (SwerveModuleState[]) b);
            }
            if (a instanceof SwerveModulePosition[] && b instanceof SwerveModulePosition[]) {
                return Arrays.equals((SwerveModulePosition[]) a, (SwerveModulePosition[]) b);
            }
            if (a instanceof ChassisSpeeds[] && b instanceof ChassisSpeeds[]) {
                return Arrays.equals((ChassisSpeeds[]) a, (ChassisSpeeds[]) b);
            }
            if (a instanceof Transform2d[] && b instanceof Transform2d[]) {
                return Arrays.equals((Transform2d[]) a, (Transform2d[]) b);
            }
            if (a instanceof Transform3d[] && b instanceof Transform3d[]) {
                return Arrays.equals((Transform3d[]) a, (Transform3d[]) b);
            }
            return false; //Unsupported array type
        }
    
        //Default object equality check
        return a.equals(b);
    }

    /**
     * Retrieves the latest value from NetworkTables, ensuring proper type safety.
     * 
     * @return The latest value from the NetworkTable, or the last known value if retrieval fails.
     */
    public synchronized T getValue() {
        Object retrievedValue = null;

        if (type == Double.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Double) lastValue);
        } else if (type == Boolean.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Boolean) lastValue);
        } else if (type == String.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (String) lastValue);
        } else if (type == double[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (double[]) lastValue);
        } else if (type == Pose2d.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Pose2d) lastValue);
        } else if (type == Pose3d.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Pose3d) lastValue);
        } else if (type == Translation2d.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Translation2d) lastValue);
        } else if (type == Pose2d[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Pose2d[]) lastValue);
        } else if (type == Pose3d[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Pose3d[]) lastValue);
        } else if (type == Translation2d[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Translation2d[]) lastValue);
        } else if (type == Translation3d.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Translation3d) lastValue);
        } else if (type == Translation3d[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Translation3d[]) lastValue);
        } else if (type == Rotation2d.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Rotation2d) lastValue);
        } else if (type == Rotation3d.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Rotation3d) lastValue);
        } else if (type == Rotation2d[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Rotation2d[]) lastValue);
        } else if (type == Rotation3d[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Rotation3d[]) lastValue);
        } else if (type == SwerveModuleState.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (SwerveModuleState) lastValue);
        } else if (type == SwerveModulePosition.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (SwerveModulePosition) lastValue);
        } else if (type == ChassisSpeeds.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (ChassisSpeeds) lastValue);
        } else if (type == Transform2d.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Transform2d) lastValue);
        } else if (type == Transform3d.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Transform3d) lastValue);
        } else if (type == Color.class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Color) lastValue);
        } else if (type == Rotation2d[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Rotation2d[]) lastValue);
        } else if (type == Rotation3d[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Rotation3d[]) lastValue);
        } else if (type == SwerveModuleState[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (SwerveModuleState[]) lastValue);
        } else if (type == SwerveModulePosition[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (SwerveModulePosition[]) lastValue);
        } else if (type == ChassisSpeeds[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (ChassisSpeeds[]) lastValue);
        } else if (type == Transform2d[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Transform2d[]) lastValue);
        } else if (type == Transform3d[].class) {
            retrievedValue = NTPublisher.retrieve(tableName, key, (Transform3d[]) lastValue);
        }

        if (retrievedValue != null && type.isInstance(retrievedValue)) {
            return type.cast(retrievedValue);
        }

        return lastValue;
    }

}
