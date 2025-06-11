package frc.robot.utils;
import java.util.function.Supplier;

import javax.lang.model.type.UnionType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Function;

//TODO: find a way to get more type errs to compile time instead of runtime
public class SendableSupplier<T> implements Supplier<T>, Sendable { //TODO; add Consumer<T> for setting sendable
    private final Supplier<T> supplier;
    private final String name;
    private Class<? extends T> expectedType; //Deal with type erasure
    private boolean typeSet = false;
    private final boolean typeInferred;
    private final Function<T, String> fallbackFormatter;
    private final boolean userDefinedFallback;


    public SendableSupplier(String name, Supplier<T> supplier) {
        this(name, supplier, null, null);
    }
    
    public SendableSupplier(String name, Supplier<T> supplier, Class<? extends T> expectedType) {
        this(name, supplier, expectedType, null);
    }
    
    public SendableSupplier(String name, Supplier<T> supplier, Class<? extends T> expectedType, Function<T, String> fallbackFormatter) {
        if (name == null || supplier == null) throw new NullPointerException("Name and supplier must not be null.");
        this.name = name;
        this.supplier = supplier;
        this.expectedType = expectedType;
        this.typeSet = (expectedType != null);
        this.typeInferred = (expectedType == null);
        this.fallbackFormatter = fallbackFormatter != null ? fallbackFormatter : Object::toString;
        this.userDefinedFallback = fallbackFormatter != null;
    }
    

    @Override
    public T get() {
        final T temp = supplier.get();
        if (temp == null) {
            DriverStation.reportWarning("Sendablesupplier: Supplierr with name '"+name+"' returned 'null'",false);
        }
        else if (this.expectedType != null && !expectedType.isInstance(temp)){ //TODO: avoid reflection 
            DriverStation.reportWarning("SendableSupplier: Supplier wiht name '"+name+"' returned a object of type "+temp.getClass().getName()+" instaed of "+expectedType.getClass().getName()+ (typeInferred?". Type was inferred":""), false);
        }
        else if (!typeSet && temp != null){
            this.expectedType = (Class<? extends T>) temp.getClass(); //safe since temp is of type T
            this.typeSet = true;
        }
        return temp;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        final T sample = supplier.get();

        if (!typeSet && sample != null) {
            this.expectedType = (Class<? extends T>) sample.getClass(); //safe since temp is of type T
            this.typeSet = true;
        }
        //TODO: expand if neede
        if (sample instanceof Boolean) {
            builder.addBooleanProperty(name, () -> (Boolean) sample, null);
        } 
        else if (sample instanceof Double || sample instanceof Float) {
            builder.addDoubleProperty(name, 
                () -> ((Number) sample).doubleValue(), null);
        } 
        else if (sample instanceof Integer || sample instanceof Long || sample instanceof Short || sample instanceof Byte) {
            builder.addIntegerProperty(name, () -> ((Number) sample).intValue(), null);
        
        } 
        else if (sample instanceof String || sample instanceof Character){
            builder.addStringProperty(name, () -> String.valueOf(sample), null);
        }
        else {
            if (!userDefinedFallback) {
                DriverStation.reportWarning("Sendabledupplier: unsupportedd type " + (sample != null ? sample.getClass().getName() : "null") + " for '" + name + "'. using fallback formatter." + (typeInferred ? " Type was infferred." : ""), false);
            }
            builder.addStringProperty(name, () -> {
                try {
                    return fallbackFormatter.apply(sample);
                } catch (Exception e) {
                    DriverStation.reportWarning("fallback formatter failed fro '" + name + "': " + e.getMessage(), false);
                    return "Fallback err";
                }
            }, null);
        }
    }
}
