package frc.robot.commands;

import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

import javax.sound.midi.*;
import java.io.File;
import java.util.*;
import java.util.concurrent.*;

public class PlayMidiCommand extends Command {
    // LED blinking parameters
    private static final int VELOCITY_THRESHOLD = 105;         // minimum velocity to trigger a blink
    private static final int ALIGNMENT_TOLERANCE_TICKS = 20;   // ticks tolerance for alignment
    private static final int[] CANDIDATE_SUBDIVISIONS = {1, 2, 4}; // quarter, eighth, sixteenth note
    private static final int DEFAULT_BLINK_DURATION_MS = 50;   // defualt
    private static final double BLINK_DURATION_RATIO = 0.4;    // blink for 40 percent of interval
    private static final int MIN_BLINK_DURATION_MS = 30;       // min blink ms
    private static final int MAX_BLINK_DURATION_MS = 350;      // max blink ms
    
    private static final int[] MOTOR_CAN_IDS = {
        2, 3, 4, 5, 6, 7, 8, 9, 31 // 9 motors
    };

    private final TalonFX[] motors;
    private final List<ScheduledNote>[] motorNoteQueues;
    private ScheduledExecutorService scheduler;
    private long startTime;

    private final LEDSubsystem m_LEDs;
    
    private SendableChooser<String> songChooser;
    private String[] songPaths;
    private String[] songNames;
    private boolean isPlaying = false;

    private Synthesizer synthesizer;
    private boolean isSimulation;

    private record ScheduledNote(double frequency, long startMs, long durationMs) {}
    private record TempoChange(long tick, int tempo) {}
    private record TimeSignature(long tick, int numerator, int denominator) {}

    private static String formatSongTitle(String filename) {
        String name = filename;
        int dot = name.lastIndexOf('.');
        if (dot > 0) {
            name = name.substring(0, dot);
        }
        name = name.replace('_', ' ');
        name = name.replaceAll("([a-z])([A-Z])", "$1 $2");

        String[] words = name.split(" ");
        for (int i = 0; i < words.length; i++) {
            if (words[i].length() > 0) {
                words[i] = words[i].substring(0, 1).toUpperCase() + words[i].substring(1).toLowerCase();
            }
        }
        return String.join(" ", words);
    }

    public PlayMidiCommand(LEDSubsystem leds) {
        this.m_LEDs = leds;
        addRequirements(leds);
        
        motors = Arrays.stream(MOTOR_CAN_IDS)
                .mapToObj(id -> new TalonFX(id, "rio"))
                .toArray(TalonFX[]::new);

        motorNoteQueues = new ArrayList[motors.length];
        for (int i = 0; i < motors.length; i++) {
            motorNoteQueues[i] = new ArrayList<>();
        }

        scheduler = Executors.newScheduledThreadPool(10);
        
        songChooser = new SendableChooser<>();
        
        scanForMidiFiles();
        
        SmartDashboard.putData("MIDI Song Selector", songChooser);
        
        isSimulation = RobotBase.isSimulation();
        SmartDashboard.putBoolean("MIDI Simulation Mode", isSimulation);
    }
    
    private void scanForMidiFiles() {
        try {
            File midiDir = new File(Filesystem.getDeployDirectory(), "midi");
            
            if (midiDir.exists() && midiDir.isDirectory()) {
                File[] files = midiDir.listFiles((dir, name) -> name.toLowerCase().endsWith(".mid"));
                
                if (files != null && files.length > 0) {
                    songPaths = new String[files.length];
                    songNames = new String[files.length];
                    
                    for (int i = 0; i < files.length; i++) {
                        songPaths[i] = "midi/" + files[i].getName();
                        songNames[i] = formatSongTitle(files[i].getName());
                    }
                    
                    for (int i = 0; i < songPaths.length; i++) {
                        if (i == 0) {
                            songChooser.setDefaultOption(songNames[i], songPaths[i]);
                        } else {
                            songChooser.addOption(songNames[i], songPaths[i]);
                        }
                    }
                } else {
                    System.err.println("No .mid files found in midi directory");
                    songPaths = new String[0];
                    songNames = new String[0];
                    songChooser.setDefaultOption("No songs found", "");
                    SmartDashboard.putString("MIDI Error", "No MIDI files found. Add .mid files to deploy/midi directory.");
                }
            } else {
                System.err.println("Midi directory not found: " + midiDir.getAbsolutePath());
                songPaths = new String[0];
                songNames = new String[0];
                songChooser.setDefaultOption("No songs found", "");
                SmartDashboard.putString("MIDI Error", "Midi directory not found: " + midiDir.getAbsolutePath());
            }
        } catch (Exception e) {
            System.err.println("Error loading midi files: " + e.getMessage());
            e.printStackTrace();
            songPaths = new String[0];
            songNames = new String[0];
            songChooser.setDefaultOption("No songs found", "");
            SmartDashboard.putString("MIDI Error", "Error loading MIDI files: " + e.getMessage());
        }
    }

    @Override
    public void initialize() {
        if (scheduler.isShutdown() || scheduler.isTerminated()) {
            scheduler = Executors.newScheduledThreadPool(10);
        }
        
        if (isSimulation) {
            try {
                synthesizer = MidiSystem.getSynthesizer();
                synthesizer.open();
                SmartDashboard.putString("MIDI Simulation", "Synthesizer initialized");
            } catch (MidiUnavailableException e) {
                System.err.println("Error initializing synthesizer: " + e.getMessage());
                SmartDashboard.putString("MIDI Simulation", "Error: " + e.getMessage());
            }
        }
        
        String selectedSong = songChooser.getSelected();
        
        if (selectedSong != null && !selectedSong.isEmpty()) {
            try {
                File midiFile = new File(Filesystem.getDeployDirectory(), selectedSong);
                Sequence sequence = MidiSystem.getSequence(midiFile);
                int resolution = sequence.getResolution(); // ticks per beat
                
                if (isSimulation && synthesizer != null && synthesizer.isOpen()) {
                    try {
                        Sequencer sequencer = MidiSystem.getSequencer(false);
                        sequencer.open();
                        sequencer.getTransmitter().setReceiver(synthesizer.getReceiver());
                        
                        sequencer.setSequence(sequence);
                        sequencer.start();
                        
                        SmartDashboard.putString("MIDI Simulation", "Playing audio through speakers");
                    } catch (MidiUnavailableException e) {
                        System.err.println("Error playing MIDI audio: " + e.getMessage());
                        SmartDashboard.putString("MIDI Simulation", "Audio error: " + e.getMessage());
                    }
                }
                
                TreeMap<Long, Integer> tempoMap = extractTempoMap(sequence);
                
                List<TimeSignature> timeSignatures = extractTimeSignatures(sequence);
                if (timeSignatures.isEmpty()) {
                    timeSignatures.add(new TimeSignature(0, 4, 4)); // default 4/4
                }
                
                //len in ticks
                long totalTicks = 0;
                for (Track track : sequence.getTracks()) {
                    for (int i = 0; i < track.size(); i++) {
                        MidiEvent event = track.get(i);
                        totalTicks = Math.max(totalTicks, event.getTick());
                    }
                }
                
                List<long[]> noteEvents = new ArrayList<>();
                for (Track track : sequence.getTracks()) {
                    extractNoteEvents(track, noteEvents);
                }
                
                int bestSubdivision = 1;
                List<Long> blinkTicks = new ArrayList<>();
                double bestScore = Double.NEGATIVE_INFINITY;
                
                for (int subdivision : CANDIDATE_SUBDIVISIONS) {
                    List<Long> timeline = buildBeatTimeline(tempoMap, timeSignatures, resolution, 
                                                           totalTicks, subdivision);
                    
                    List<Long> candidateBlinkTicks = new ArrayList<>();
                    Map<Long, Integer> velocityPerSubdiv = new HashMap<>();
                    
                    for (long[] event : noteEvents) {
                        long noteTick = event[0];
                        int velocity = (int)event[1];
                        
                        long closestBeat = findClosestBeat(timeline, noteTick);
                        if (Math.abs(closestBeat - noteTick) <= ALIGNMENT_TOLERANCE_TICKS) {
                            velocityPerSubdiv.put(closestBeat, 
                                                 velocityPerSubdiv.getOrDefault(closestBeat, 0) + velocity);
                        }
                    }
                    
                    for (Map.Entry<Long, Integer> entry : velocityPerSubdiv.entrySet()) {
                        if (entry.getValue() >= VELOCITY_THRESHOLD) {
                            candidateBlinkTicks.add(entry.getKey());
                        }
                    }
                    
                    if (!candidateBlinkTicks.isEmpty()) {
                        Collections.sort(candidateBlinkTicks);
                        
                        double variance = 0;
                        if (candidateBlinkTicks.size() > 1) {
                            List<Long> intervals = new ArrayList<>();
                            for (int i = 1; i < candidateBlinkTicks.size(); i++) {
                                intervals.add(candidateBlinkTicks.get(i) - candidateBlinkTicks.get(i-1));
                            }
                            
                            double avgInterval = intervals.stream().mapToLong(v -> v).average().orElse(0);
                            variance = intervals.stream()
                                .mapToDouble(interval -> Math.pow(interval - avgInterval, 2))
                                .sum() / intervals.size();
                        }
                        
                        double score = candidateBlinkTicks.size() - 0.0005 * variance;
                        
                        if (score > bestScore) {
                            bestScore = score;
                            bestSubdivision = subdivision;
                            blinkTicks = candidateBlinkTicks;
                        }
                    }
                }
                
                SmartDashboard.putNumber("LED Beat Subdivision", bestSubdivision);
                SmartDashboard.putNumber("LED Blink Count", blinkTicks.size());
                
                for (List<ScheduledNote> queue : motorNoteQueues) {
                    queue.clear();
                }

                Track[] tracks = sequence.getTracks();
                for (int i = 0; i < Math.min(motors.length, tracks.length); i++) {
                    extractNotesFromTrack(tracks[i], resolution, tempoMap, motorNoteQueues[i]);
                }
                
                startTime = System.currentTimeMillis();
                isPlaying = true;
                
                for (int i = 0; i < motors.length; i++) {
                    final TalonFX motor = motors[i];
                    for (ScheduledNote note : motorNoteQueues[i]) {
                        scheduler.schedule(() -> {
                            motor.setControl(new MusicTone(note.frequency));
                            scheduler.schedule(() -> motor.setControl(new MusicTone(0)),
                                    note.durationMs, TimeUnit.MILLISECONDS);
                        }, note.startMs, TimeUnit.MILLISECONDS);
                    }
                }
                
                for (int i = 0; i < blinkTicks.size(); i++) {
                    Long blinkTick = blinkTicks.get(i);
                    long blinkTimeMs = (long)tickToMs(blinkTick, resolution, tempoMap);
                    
                    long blinkDurationMs;
                    if (i < blinkTicks.size() - 1) {
                        Long nextBlinkTick = blinkTicks.get(i + 1);
                        long nextBlinkTimeMs = (long)tickToMs(nextBlinkTick, resolution, tempoMap);
                        long intervalMs = nextBlinkTimeMs - blinkTimeMs;
                        
                        blinkDurationMs = (long)(intervalMs * BLINK_DURATION_RATIO);
                        
                        blinkDurationMs = Math.max(MIN_BLINK_DURATION_MS, 
                                          Math.min(MAX_BLINK_DURATION_MS, blinkDurationMs));
                    } else {
                        blinkDurationMs = DEFAULT_BLINK_DURATION_MS;
                    }
                    
                    boolean isStrongBeat = isStrongBeat(blinkTick, timeSignatures, resolution);
                    double blinkColor = isStrongBeat ? 
                                      LEDSubsystem.LEDConstants.colorRed : 
                                      LEDSubsystem.LEDConstants.colorBlue;
                    
                    final long finalBlinkDurationMs = blinkDurationMs;
                    
                    scheduler.schedule(() -> {
                        m_LEDs.blink(finalBlinkDurationMs, blinkColor, LEDSubsystem.LEDConstants.colorOff);
                    }, blinkTimeMs, TimeUnit.MILLISECONDS);
                    
                    if (i == 0) {
                        SmartDashboard.putNumber("First Blink Duration", blinkDurationMs);
                    }
                }
                
                SmartDashboard.putString("MIDI load status", "Playing: " + selectedSong);
            } catch (Exception e) {
                System.err.println("Error loading MIDI: " + e.getMessage());
                e.printStackTrace();
                SmartDashboard.putString("MIDI load status", "Error: " + e.getMessage());
                isPlaying = false;
                safeShutdownScheduler();
            }
        } else {
            SmartDashboard.putString("MIDI load status", "No song selected");
            isPlaying = false;
        }
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Current MIDI", songChooser.getSelected());
        SmartDashboard.putBoolean("Is Playing", isPlaying);
    }

    @Override
    public void end(boolean interrupted) {
        for (TalonFX motor : motors) {
            motor.setControl(new MusicTone(0)); // silence all motors
        }
        
        if (isSimulation && synthesizer != null && synthesizer.isOpen()) {
            synthesizer.close();
        }
        
        safeShutdownScheduler(); // cancel pending events
        isPlaying = false;
    }

    private void safeShutdownScheduler() {
        if (scheduler != null && !scheduler.isShutdown()) {
            try {
                scheduler.shutdown();
                
                if (!scheduler.awaitTermination(100, TimeUnit.MILLISECONDS)) {
                    scheduler.shutdownNow(); //force shutdown because my code doesnt work
                }
            } catch (InterruptedException e) {
                scheduler.shutdownNow();
                Thread.currentThread().interrupt(); 
            } catch (Exception e) {
                System.err.println("Error shutting down scheduler: " + e.getMessage());
                e.printStackTrace();
                
                try {
                    scheduler.shutdownNow();
                } catch (Exception ignored) {
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false; // play until interrupted
    }

    // cleanup
    @Override
    protected void finalize() throws Throwable {
        try {
            safeShutdownScheduler();
        } finally {
            super.finalize();
        }
    }

    private static TreeMap<Long, Integer> extractTempoMap(Sequence seq) {
        TreeMap<Long, Integer> tempoMap = new TreeMap<>();
        tempoMap.put(0L, 500_000); // default 120 bpm
        for (Track track : seq.getTracks()) {
            for (int i = 0; i < track.size(); i++) {
                MidiEvent event = track.get(i);
                MidiMessage msg = event.getMessage();
                if (msg instanceof MetaMessage meta && meta.getType() == 0x51) {
                    byte[] data = meta.getData();
                    int tempo = ((data[0] & 0xFF) << 16)
                              | ((data[1] & 0xFF) << 8)
                              | (data[2] & 0xFF);
                    tempoMap.put(event.getTick(), tempo);
                }
            }
        }
        return tempoMap;
    }

    private static void extractNotesFromTrack(Track track, int resolution, TreeMap<Long, Integer> tempoMap, List<ScheduledNote> notes) {
        Map<Integer, Long> noteStartTicks = new HashMap<>();

        for (int i = 0; i < track.size(); i++) {
            MidiEvent event = track.get(i);
            MidiMessage msg = event.getMessage();
            long tick = event.getTick();

            if (msg instanceof ShortMessage sm) {
                int cmd = sm.getCommand();
                int note = sm.getData1();
                int velocity = sm.getData2();

                if (cmd == ShortMessage.NOTE_ON && velocity > 0) {
                    noteStartTicks.put(note, tick);
                } else if (cmd == ShortMessage.NOTE_OFF || (cmd == ShortMessage.NOTE_ON && velocity == 0)) {
                    Long startTick = noteStartTicks.remove(note);
                    if (startTick != null) {
                        long startMs = (long) tickToMs(startTick, resolution, tempoMap);
                        long endMs = (long) tickToMs(tick, resolution, tempoMap);
                        double freq = 440.0 * Math.pow(2, (note - 69) / 12.0);
                        notes.add(new ScheduledNote(freq, startMs, endMs - startMs));
                    }
                }
            }
        }
    }

    private static double tickToMs(long tick, int resolution, TreeMap<Long, Integer> tempoMap) {
        double timeMs = 0;
        long prevTick = 0;
        for (Map.Entry<Long, Integer> entry : tempoMap.entrySet()) {
            long tempoTick = entry.getKey();
            if (tempoTick >= tick) break;
            int tempo = entry.getValue();
            long dt = tempoTick - prevTick;
            double msPerTick = tempo / 1000.0 / resolution;
            timeMs += dt * msPerTick;
            prevTick = tempoTick;
        }

        int lastTempo = tempoMap.floorEntry(tick).getValue();
        double msPerTick = lastTempo / 1000.0 / resolution;
        timeMs += (tick - prevTick) * msPerTick;
        return timeMs;
    }

    private boolean isStrongBeat(long tick, List<TimeSignature> timeSignatures, int resolution) {
        TimeSignature ts = null;
        for (int i = timeSignatures.size() - 1; i >= 0; i--) {
            if (timeSignatures.get(i).tick <= tick) {
                ts = timeSignatures.get(i);
                break;
            }
        }
        
        if (ts == null) return false;
        
        int measureLengthTicks = resolution * 4 * ts.numerator / ts.denominator;
        
        long ticksSinceTimeSig = tick - ts.tick;
        long positionInMeasure = ticksSinceTimeSig % measureLengthTicks;
        
        return positionInMeasure < ALIGNMENT_TOLERANCE_TICKS;
    }
    
    private long findClosestBeat(List<Long> timeline, long tick) {
        return timeline.stream()
            .min(Comparator.comparingLong(t -> Math.abs(t - tick)))
            .orElse(0L);
    }
    
    private void extractNoteEvents(Track track, List<long[]> noteEvents) {
        for (int i = 0; i < track.size(); i++) {
            MidiEvent event = track.get(i);
            MidiMessage msg = event.getMessage();
            
            if (msg instanceof ShortMessage sm) {
                int cmd = sm.getCommand();
                int velocity = sm.getData2();
                
                if (cmd == ShortMessage.NOTE_ON && velocity > 0) {
                    noteEvents.add(new long[]{event.getTick(), velocity});
                }
            }
        }
    }
    
    private List<TimeSignature> extractTimeSignatures(Sequence seq) {
        List<TimeSignature> signatures = new ArrayList<>();
        
        for (Track track : seq.getTracks()) {
            for (int i = 0; i < track.size(); i++) {
                MidiEvent event = track.get(i);
                MidiMessage msg = event.getMessage();
                
                if (msg instanceof MetaMessage meta && meta.getType() == 0x58) {
                    byte[] data = meta.getData();
                    int numerator = data[0];
                    int denominator = (int) Math.pow(2, data[1]);
                    signatures.add(new TimeSignature(event.getTick(), numerator, denominator));
                }
            }
        }
        
        return signatures;
    }
    
    private List<Long> buildBeatTimeline(TreeMap<Long, Integer> tempoMap, List<TimeSignature> timeSignatures,
                                         int ticksPerBeat, long totalTicks, int subdivisionsPerBeat) {
        List<Long> timeline = new ArrayList<>();
        long currentTick = 0;
        
        while (currentTick < totalTicks) {
            Map.Entry<Long, Integer> tempoEntry = tempoMap.floorEntry(currentTick);
            int tempo = tempoEntry != null ? tempoEntry.getValue() : 500000; // Default 120 BPM
            
            TimeSignature ts = null;
            for (int i = timeSignatures.size() - 1; i >= 0; i--) {
                if (timeSignatures.get(i).tick <= currentTick) {
                    ts = timeSignatures.get(i);
                    break;
                }
            }
            if (ts == null) ts = new TimeSignature(0, 4, 4); //default 4/4
            
            int beatLengthTicks = ticksPerBeat * 4 / ts.denominator;
            int subdivLengthTicks = beatLengthTicks / subdivisionsPerBeat;
            
            for (int i = 0; i < subdivisionsPerBeat; i++) {
                long tick = currentTick + i * subdivLengthTicks;
                if (tick < totalTicks) {
                    timeline.add(tick);
                }
            }
            
            currentTick += beatLengthTicks;
        }
        
        return timeline;
    }
}
