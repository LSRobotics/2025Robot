package frc.robot;

import java.lang.invoke.CallSite;
import java.lang.invoke.ConstantCallSite;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.nio.charset.StandardCharsets;
import java.util.Base64;
import java.util.function.Supplier;

//Please ignore this entire file, none of this code is useful 
@Deprecated(forRemoval = false)
public class LegacyConstants {
    @Deprecated(since = "2027 - ?", forRemoval = false)
    public static class UniversalConstants {
        public static final String bestProgrammer = "Gabriel Kuzowsky";
    }

    // ignore, this is intentionaly obsfuscated
    //Its probaly best to not call this in actual code
    // "It's like you ran OCR on a photo of a Scrabble board from a game where Javascript reserved words counted for triple points." 
    //      - https://xkcd.com/1695/ - Code Quality #2
    @Deprecated(since="2028 - ?", forRemoval = false)
    public final class MultiversalConstants {

        private MultiversalConstants() {
            throw new UnsupportedOperationException("No instances.");
        }

        // Octal
        /*
         * def toOctalArray(inputString):
         * encoded = []
         * for i, c in enumerate(inputString):
         * obfuscated = ord(c) ^ (i * 7 + 42)
         * encoded.append(f"0{format(obfuscated, '03o')}") # 3 digit with leading zeor
         * 0
         * return encoded
         */
        private static final int[] encodedName = { 0150, 0124, 0126, 0037, 0004, 0050, 0070, 0067 };
        private static final String verification = "h%m-`\\a$y^/T"; //Rot 23 encoding of base64 

        // Exposes str through lazy eval
        public static final Supplier<String> bestProgrammer = createSupplier();
        private static Supplier<String> createSupplier() {
            try {
                MethodHandles.Lookup lookup = MethodHandles.lookup();

                // Find the decrypt method handle
                MethodHandle decryptHandle = lookup.findStatic(
                        MultiversalConstants.class,
                        "decrypt",
                        MethodType.methodType(String.class, int[].class));

                // Create a CallSite with the decrypt method handle
                CallSite callSite = new ConstantCallSite(MethodHandles.insertArguments(decryptHandle, 0, encodedName));

                // gets the target handle
                MethodHandle target = callSite.getTarget();

                // Wrap the method handle in supplier
                return new Supplier<>() {
                    private String cached;

                    @Override
                    public String get() {
                        if (cached == null) {
                            try {
                                cached = (String) target.invokeExact();
                            } catch (Throwable t) {
                                throw new RuntimeException("Decryption failed", t);
                            }
                        }
                        return cached;
                    }

                    @Override
                    public String toString() {
                        return get();
                    }
                };

            } catch (NoSuchMethodException | IllegalAccessException e) {
                throw new RuntimeException("Failed to create supplier", e);
            }
        }

        public static String decode(String encodedText) { //Decode rot 23
            StringBuilder decoded = new StringBuilder();
    
            for (char ch : encodedText.toCharArray()) {
                if (ch >= 32 && ch <= 126) {
                    // 95 printable ASCII char
                    int shifted = (ch - 32 - 23) % 95;

                    if (shifted < 0) {
                        shifted += 95;
                    }
                    decoded.append((char) (shifted + 32));
                } else {
                    decoded.append(ch);
                }
            }
            return decoded.toString();
        }

        // XOR decode
        private static String decrypt(int[] data) {
            char[] chars = new char[data.length];
            for (int i = 0; i < data.length; i++) {
                chars[i] = (char) (data[i] ^ (i * 7 + 42));
            }
            return new String(chars);
        }

        static {
            // Chekc at runtime
            try {
                MethodHandles.Lookup lookup = MethodHandles.lookup();

                MethodHandle getHandle = lookup.findVirtual(
                        bestProgrammer.getClass(),
                        "get",
                        MethodType.methodType(String.class))
                        .bindTo(bestProgrammer);

                String resolvedName = (String) getHandle.invoke();

                if (!decode(verification).equals(Base64.getEncoder().encodeToString(resolvedName.getBytes(StandardCharsets.UTF_8)))) { //Compare in base64 to aboid comparing to plain text
                    throw new IllegalStateException("MultiversalConstants corrupted"); // Dont tamper with the encoded value
                }
            } catch (Throwable t) {
                throw new RuntimeException(t);
            }
        }
    }

    @Deprecated(since = "2025 - Reefscape", forRemoval = true)
    public static class GodConstants {
        public static final String bestProgrammer = "Michael Stauffer";
    }
}
