package frc.robot.utils;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
public class SendableSupplier<T> implements Supplier<T>, Sendable {
    private final Supplier<T> supplier;
    private final String name;
    private Class<? extends T> expectedType; //Deal with type erasure
    private boolean typeSet = false;

    public SendableSupplier(String name, Supplier<T> supplier) {
        this.supplier = supplier;
        this.name = name;
        this.expectedType = null;
    }

    public SendableSupplier(String name, Supplier<T> supplier, Class<? extends T> expectedType) {
        this.supplier = supplier;
        this.name = name;
        this.expectedType = expectedType;
        this.typeSet = true;
    }

    @Override
    public T get() {
        T temp = supplier.get();
        if (temp == null) {
            DriverStation.reportWarning("Sendablesupplier: Supplierr with name '"+name+"' returned 'null'",false);
        }
        else if (this.expectedType != null && !expectedType.isInstance(temp)){
            DriverStation.reportWarning("SendableSupplier: Supplier wiht name '"+name+"' returned a object of type "+temp.getClass().getName()+" instaed of "+expectedType.getClass().getName(), false);
        }
        else if (!typeSet && temp != null){
            this.expectedType = (Class<? extends T>) temp.getClass(); //safe since temp is of type T
            this.typeSet = true;
        }
        return temp;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        T sample = supplier.get();

        if (!typeSet && sample != null) {
            this.expectedType = (Class<? extends T>) sample.getClass(); //safe since temp is of type T
            this.typeSet = true;
        }

        if (sample instanceof Boolean) {
            builder.addBooleanProperty(name, () -> (Boolean) supplier.get(), null);
        } 
        else if (sample instanceof Double || sample instanceof Float) {
            builder.addDoubleProperty(name, 
                () -> ((Number) supplier.get()).doubleValue(), null);
        } 
        else if (sample instanceof Integer || sample instanceof Long || sample instanceof Short || sample instanceof Byte) {
            builder.addIntegerProperty(name, () -> ((Number) supplier.get()).intValue(), null);
        
        } 
        else if (sample instanceof String || sample instanceof Character){
            builder.addStringProperty(name, () -> String.valueOf(supplier.get()), null);
        }
        else {
            DriverStation.reportWarning("sendableSupplier: unsuppported type " + (sample != null ? sample.getClass().getName() : "null") + ". falling back to string",false);
            builder.addStringProperty(name, () -> String.valueOf(supplier.get()), null);
        }
    }
}
