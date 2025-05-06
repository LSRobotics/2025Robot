package frc.robot.utils;
import java.util.function.Supplier;
import java.util.jar.Attributes.Name;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableSupplier<T> implements Supplier<T>, Sendable {
    private final Supplier<T> supplier;
    private final String name;

    public SendableSupplier(String name, Supplier<T> supplier) {
        this.supplier = supplier;
        this.name = name;
    }

    @Override
    public T get() {
        return supplier.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        T sample = supplier.get();
        
        if (sample instanceof Boolean) {
            builder.addBooleanProperty(name, 
                () -> (Boolean) supplier.get(), 
                null);
        } else if (sample instanceof Double || sample instanceof Float) {
            builder.addDoubleProperty(name, 
                () -> ((Number) supplier.get()).doubleValue(), 
                null);
        } else if (sample instanceof Integer || sample instanceof Long) {
            builder.addIntegerProperty(name, 
                () -> ((Number) supplier.get()).intValue(), 
                null);
        } else {
            // Default to string 
            builder.addStringProperty(name, 
                () -> String.valueOf(supplier.get()), 
                null);
        }
    }
}
