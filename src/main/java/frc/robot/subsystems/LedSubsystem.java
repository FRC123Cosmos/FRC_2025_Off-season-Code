// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Microseconds;
// import static edu.wpi.first.units.Units.Percent;
// import static edu.wpi.first.units.Units.Second;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.LEDPattern;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.LEDConstants;

// // public class LedSubsystem extends SubsystemBase {
// //     private final AddressableLED ledBar;
// //     private LEDPattern allianceLED;

// //     private enum LedState { BLANK, RAINBOW, SCROLL, BREATHING, ALLIANCE_SOLID, GREEN, YELLOW }
// //     private LedState currentState = LedState.BLANK;

// //     private boolean hasCoral = false; 
// //     private boolean ongoingCoral = false; 

// //     public LedSubsystem() {
// //         // Initialize alliance color
// //         setAllianceColor();

// //         // Initialize LED hardware
// //         ledBar = new AddressableLED(LEDConstants.kLEDBarPWM);
// //         ledBar.setLength(LEDConstants.ledLength);

// //         // Start with blank pattern
// //         setBlank();
// //         ledBar.start();

// //         // Register this subsystem with the command scheduler
// //         this.register(); // Ensures periodic() is called automatically
// //     }

// //     @Override
// //     public void periodic() {
// //         // Update LED state based on robot mode and conditions
// //         if (DriverStation.isDisabled()) {
// //             setScroll();
// //         } else if (DriverStation.isAutonomous()) {
// //             setRainbow();
// //         } else if (DriverStation.isTeleop()) {
// //             // double timeRemaining = DriverStation.getMatchTime();
// //             // if (timeRemaining > 0 && timeRemaining < 20) {
// //             //     setBreathing();
// //             // } else if (hasCoral) {
// //             //     setGreen();
// //             // } else if (ongoingCoral) { 
// //             //     setYellow();
// //             // } else {
// //                 setAllianceSolid();
// //             // }
// //         }
// //     }

// //     private void setAllianceColor() {
// //         DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
// //         allianceLED = (alliance == DriverStation.Alliance.Red)
// //             ? LEDPattern.solid(Color.kRed)
// //             : LEDPattern.solid(Color.kBlue);
// //     }

// //     // Helper method to create a new buffer
// //     private AddressableLEDBuffer createBuffer() {
// //         return new AddressableLEDBuffer(LEDConstants.ledBufferLength);
// //     }

// //     // Public methods with individual buffers
// //     public void setBlank() {
// //         if (currentState != LedState.BLANK) {
// //             AddressableLEDBuffer buffer = createBuffer();
// //             LEDPattern pattern = LEDPattern.solid(Color.kBlack);
// //             pattern.applyTo(buffer);
// //             ledBar.setData(buffer);
// //             currentState = LedState.BLANK;
// //         }
// //     }

// //     public void setRainbow() {
// //         if (currentState != LedState.RAINBOW) {
// //             AddressableLEDBuffer buffer = createBuffer();
// //             LEDPattern pattern = LEDPattern.rainbow(255, 128).atBrightness(Percent.of(100));
// //             pattern.applyTo(buffer);
// //             ledBar.setData(buffer);
// //             currentState = LedState.RAINBOW;
// //         }
// //     }

// //     public void setScroll() {
// //         if (currentState != LedState.SCROLL) {
// //             AddressableLEDBuffer buffer = createBuffer();
// //             LEDPattern pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kCoral, Color.kWheat)
// //                 .scrollAtRelativeSpeed(Percent.per(Second).of(25));
// //             pattern.applyTo(buffer);
// //             ledBar.setData(buffer);
// //             currentState = LedState.SCROLL;
// //         }
// //     }

// //     public void setBreathing() {
// //         if (currentState != LedState.BREATHING) {
// //             AddressableLEDBuffer buffer = createBuffer();
// //             LEDPattern pattern = allianceLED.breathe(Second.of(2.5));
// //             pattern.applyTo(buffer);
// //             ledBar.setData(buffer);
// //             currentState = LedState.BREATHING;
// //         }
// //     }

// //     public void setAllianceSolid() {
// //         if (currentState != LedState.ALLIANCE_SOLID) {
// //             AddressableLEDBuffer buffer = createBuffer();
// //             LEDPattern pattern = allianceLED;
// //             pattern.applyTo(buffer);
// //             ledBar.setData(buffer);
// //             currentState = LedState.ALLIANCE_SOLID;
// //         }
// //     }

// //     public void setGreen() {
// //         if (currentState != LedState.GREEN) {
// //             AddressableLEDBuffer buffer = createBuffer();
// //             LEDPattern pattern = LEDPattern.solid(Color.kGreen);
// //             pattern.applyTo(buffer);
// //             ledBar.setData(buffer);
// //             currentState = LedState.GREEN;
// //         }
// //     }

// //     public void setYellow() {
// //         if (currentState != LedState.YELLOW) {
// //             AddressableLEDBuffer buffer = createBuffer();
// //             LEDPattern pattern = LEDPattern.solid(Color.kYellow);
// //             pattern.applyTo(buffer);
// //             ledBar.setData(buffer);
// //             currentState = LedState.YELLOW;
// //         }
// //     }

// //     // Removed setElevatorProgress() since it depended on ElevatorSubsystem

// //     public void stop() {
// //         ledBar.stop();
// //     }

// //     public void start() {
// //         ledBar.start();
// //     }

// //     private static final LedSubsystem instance = new LedSubsystem();

// //     public static LedSubsystem getInstance() {
// //         return instance;
// //     }
// // }
// public class LedSubsystem extends SubsystemBase {

//     // private static ElevatorSubsystem elevator = new ElevatorSubsystem();
            
//             private static AddressableLED ledBar; // instance of the led bar class

//             private static double allianceBlinkLastChangeTime = 0;
//             private static boolean allianceBlinkIsOn = true;
//             private static final double ALLIANCE_BLINK_INTERVAL = 0.25; // 250ms per phase (2 Hz full cycle)
//             private static final double ALLIANCE_BLINK_INTERVAL_SLOW = 1;
//             // Buffer or the holder of the color's data and its variable
//             private static AddressableLEDBuffer led_red_alliance;
//             private static AddressableLEDBuffer led_blue_alliance;
//             private static AddressableLEDBuffer led_red_blue;
//             private static AddressableLEDBuffer led_blank;
//             private static AddressableLEDBuffer led_green;
//             private static AddressableLEDBuffer led_yellow;
//             private static AddressableLEDBuffer led_orange;
//             private static AddressableLEDBuffer led_purple;
//             // private static AddressableLEDBuffer led_dynamic_msg;
//             private static AddressableLEDBuffer rainbow_buffer;
//             private static AddressableLEDBuffer elevator_progress_buffer;
//             private static AddressableLEDBuffer scroll_buffer;
//             private static AddressableLEDBuffer breath_buffer;
//             private static AddressableLEDBuffer alliance_buffer;
//             private static AddressableLEDBuffer autonomous_buffer;
//             private static AddressableLEDBuffer policeLed_buffer;
//             private static AddressableLEDBuffer blink_alliance_buffer;
//             private static AddressableLEDBuffer checkerBoard_buffer;
//             private static AddressableLEDBuffer counterChase_buffer;
//             private static AddressableLEDBuffer cosmic_buffer;

//             private static LEDPattern scrollBase = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kCyan, Color.kTeal);
//             private static LEDPattern allianceLED;
        
//             private static double policeLastChangeTime = 0;
//             private static boolean policeIsFirstPhase = true;
//             private static final double POLICE_BLINK_INTERVAL = 0.05; // 150ms per phase (~6.67 Hz cycle)
//             private static final int HALF_LENGTH = LEDConstants.ledLength / 2;
        
//             // private static int dynamicCount = 0; // counter for dynamic messages
//             // private static int rainbowFirstPixelHue = 0; // tracks the hue for the first pixel in the rainbow effect
//             // public static boolean dynamicEffect = false; // a tracking varible for checking dynamic effect
        
//             // a block that runs once when class is initially loaded
//             static {
//                 setAllianceColor();

//                 ledBar = new AddressableLED(LEDConstants.kLEDBarPWM); // initializes the led bar object with the given power port
//                 ledBar.setLength(LEDConstants.ledLength);
        
//                 // set buffers to their specified length
        
//                 led_red_alliance = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_blue_alliance = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_red_blue = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_blank = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 rainbow_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 elevator_progress_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 scroll_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 breath_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 alliance_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_green = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_yellow = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_orange = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 autonomous_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 policeLed_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 blink_alliance_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 checkerBoard_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 counterChase_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 cosmic_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_purple = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 // led_dynamic_msg = new AddressableLEDBuffer(LEDConstants.ledBufferLength); // specific blinking message
        
//                 // initial message for led buffers
//                 for (int i = 0 ; i < LEDConstants.ledLength; i++){
//                     // pre-set all message buffers during initilization
//                     led_red_alliance.setLED(i, Color.kRed);
//                     led_blue_alliance.setLED(i, Color.kBlue);
//                     led_green.setLED(i, Color.kGreen);
//                     led_yellow.setLED(i, Color.kYellow);
//                     led_orange.setLED(i, Color.kOrange);
//                     led_purple.setLED(i, Color.kPurple);
        
//                     // using bitwise-AND operators .... (alternating red-blue led)
//                     if (((i & 3) == 0) // if the last two binary value of i & 3 is the same as 0's
//                      || ((i & 3) == 2) // if if the last two binary value of i & 3 is the same as 2's
//                      ){
//                         led_red_blue.setLED(i, Color.kRed);
//                     }  else {
//                         led_red_blue.setLED(i, Color.kBlue);
//                     }
        
//                     led_blank.setLED(i, Color.kBlack);
//                     // led_dynamic_msg.setLED(i, Color.kBlack); // no pre-set for dynamic msg
//                 }
        
//                 ledBar.setData(led_blank); // displays the black-led at first
//                 ledBar.start(); // activates the led strip
//             }
        
//             public static void stopLedBar(){
//                 ledBar.stop(); // turns off
//             }
        
//             public static void startLedBar(){
//                 ledBar.start(); // turns on
//             }
        
//             public static void setBlankMsg(){
//                 // dynamicEffect = false;
//                 ledBar.setData(led_blank); // doesn't light up
//             }
        
//             public static void setRedBlueMsg(){
//                 // dynamicEffect = false; 
//                 ledBar.setData(led_red_blue); // lights up red_blue
//             }
        
//             public static void setGreenMsg(){
//                 // dynamicEffect = false;
//                 ledBar.setData(led_green); // lights up green
//             }

//             public static void setPurpleMsg(){
//                 // dynamicEffect = false;
//                 ledBar.setData(led_purple); // lights up green
//             }

//             public static void setOrangeMsg(){
//                 ledBar.setData(led_orange);
//             }

//             public static void setYellowMsg(){
//                 // dynamicEffect = false;
//                 ledBar.setData(led_yellow); // lights up green
//             }
        
//             public static void setAllianceColor(){ // sets allaince color in led
//                 DriverStation.Alliance friendlyAlliance = DriverStation.getAlliance().get();
//                 if(friendlyAlliance == DriverStation.Alliance.Red){
//                     allianceLED = LEDPattern.solid(Color.kRed);
//                 }
//                 else if(friendlyAlliance == DriverStation.Alliance.Blue){
//                     allianceLED = LEDPattern.solid(Color.kBlue);
//                 }
//                 else {
//                     allianceLED = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
//                 }
//             }

//             public static void blinkAllianceSolid() {
//                 double currentTime = Timer.getFPGATimestamp();
        
//                 // Switch phases if enough time has passed
//                 if (currentTime - allianceBlinkLastChangeTime >= ALLIANCE_BLINK_INTERVAL) {
//                     allianceBlinkIsOn = !allianceBlinkIsOn;
//                     allianceBlinkLastChangeTime = currentTime;
//                     // Optional: Debug output
//                     System.out.println("Alliance blink phase: " + (allianceBlinkIsOn ? "ON" : "OFF") + " at time: " + currentTime);
//                 }
        
//                 if (allianceBlinkIsOn) {
//                     // Set solid alliance color
//                     allianceLED.applyTo(alliance_buffer);
//                     ledBar.setData(alliance_buffer);
//                 } else {
//                     // Turn off (black)
//                     ledBar.setData(led_blank);
//                 }
//             }

//             public static void blinkAllianceSolidSlow() {
//                 double currentTime = Timer.getFPGATimestamp();
        
//                 // Switch phases if enough time has passed
//                 if (currentTime - allianceBlinkLastChangeTime >= ALLIANCE_BLINK_INTERVAL_SLOW) {
//                     allianceBlinkIsOn = !allianceBlinkIsOn;
//                     allianceBlinkLastChangeTime = currentTime;
//                     // Optional: Debug output
//                     System.out.println("Alliance blink phase: " + (allianceBlinkIsOn ? "ON" : "OFF") + " at time: " + currentTime);
//                 }
        
//                 if (allianceBlinkIsOn) {
//                     // Set solid alliance color
//                     allianceLED.applyTo(alliance_buffer);
//                     ledBar.setData(alliance_buffer);
//                 } else {
//                     // Turn off (black)
//                     ledBar.setData(led_blank);
//                 }
//             }

//             public static void setAllianceBlink() {
//                 // Use allianceLED and apply a fast blink (0.2s cycle = 5 Hz)
//                 LEDPattern blinkPattern = allianceLED.blink(Second.of(0.2)).atBrightness(Percent.of(100));
                
//                 // Apply to the blink buffer and update LED strip
//                 blinkPattern.applyTo(blink_alliance_buffer);
//                 ledBar.setData(blink_alliance_buffer);
//             }

//             public static void setAllianceSolid(){
//                 allianceLED.applyTo(alliance_buffer);
//                 ledBar.setData(alliance_buffer);
//                 ledBar.setData(alliance_buffer);
//             }
//             public static void setAutonomousPattern() {
//                 // Base layer: Dim gold background
//                 LEDPattern baseLayer = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(20));

//                 // Chase layer: Bright purple pulses moving across the strip
//                 LEDPattern chaseLayer = LEDPattern.solid(Color.kBlue)
//                     .breathe(Second.of(.5)) // Pulse every 1 second
//                     .scrollAtRelativeSpeed(Percent.per(Second).of(50)) // Move at 50% speed
//                     .atBrightness(Percent.of(80)); // Bright pulses

//                 // Combine layers: Overlay chase on base
//                 LEDPattern autonomousPattern = chaseLayer.overlayOn(baseLayer);

//                 // Apply to buffer and update LED strip
//                 autonomousPattern.applyTo(autonomous_buffer);
//                 ledBar.setData(autonomous_buffer);
//             }
        
//             public static void scrollMsg(){
//                 LEDPattern pattern = scrollBase.scrollAtRelativeSpeed(Percent.per(Second).of(50));
        
//                 pattern.applyTo(scroll_buffer);
        
//                 ledBar.setData(scroll_buffer);
//             }
        
//             public static void setBreathingMsg(){
//                 LEDPattern breathing = allianceLED.breathe(Second.of(
//                 2)).atBrightness(Percent.of(50));
        
//                 breathing.applyTo(breath_buffer);
        
//                 ledBar.setData(breath_buffer);
//             }
        
//             public static void setRainbow(){
//                 LEDPattern rainbow = LEDPattern.rainbow(255, 128).atBrightness(Percent.of(100));
//                 rainbow.applyTo(rainbow_buffer);
//                 ledBar.setData(rainbow_buffer);
//             }

//             public static void setPoliceLights() {
//                 double currentTime = Timer.getFPGATimestamp();

//                 // Switch phases if enough time has passed
//                 if (currentTime - policeLastChangeTime >= POLICE_BLINK_INTERVAL) {
//                     policeIsFirstPhase = !policeIsFirstPhase;
//                     policeLastChangeTime = currentTime;
//                     // Optional: Add debug output to verify phase switching
//                     System.out.println("Phase switched to: " + (policeIsFirstPhase ? "First" : "Second") + " at time: " + currentTime);
//                 }

//                 // Set alternating patterns for each half
//                 for (int i = 0; i < LEDConstants.ledLength; i++) {
//                     if (policeIsFirstPhase) {
//                         // First phase: First half red, second half blue
//                         if (i < HALF_LENGTH) {
//                             policeLed_buffer.setLED(i, Color.kRed);
//                         } else {
//                             policeLed_buffer.setLED(i, Color.kBlue);
//                         }
//                     } else {
//                         // Second phase: First half blue, second half red
//                         if (i < HALF_LENGTH) {
//                             policeLed_buffer.setLED(i, Color.kBlue);
//                         } else {
//                             policeLed_buffer.setLED(i, Color.kRed);
//                         }
//                     }
//                 }

//                 // Update LED strip
//                 ledBar.setData(policeLed_buffer);
//             }
            
//             public static void setBinaryCheckerboard() {
//                 for (int i = 0; i < LEDConstants.ledLength; i++) {
//                     // Use bitwise AND with 1 to alternate every other LED
//                     if ((i & 1) == 0) {
//                         checkerBoard_buffer.setLED(i, Color.kPurple); // Even indices
//                     } else {
//                         checkerBoard_buffer.setLED(i, Color.kBlack);  // Odd indices
//                     }
//                 }
//                 ledBar.setData(checkerBoard_buffer);
//             }

//             private static int binaryCounter = 0;

//             public static void setBinaryCounterChase() {
//                 for (int i = 0; i < LEDConstants.ledLength; i++) {
//                     // Use bitwise AND with a shifted counter
//                     if (((i + binaryCounter) & 7) < 4) { // Mask with 7 (111 in binary), threshold at 4
//                         counterChase_buffer.setLED(i, Color.kCyan);
//                     } else {
//                         counterChase_buffer.setLED(i, Color.kBlack);
//                     }
//                 }
//                 ledBar.setData(counterChase_buffer);

//                 // Increment counter for animation (update every ~20ms in periodic())
//                 binaryCounter = (binaryCounter + 1) % LEDConstants.ledLength;
//             }

//             public static void setCosmicBinaryStorm() {
//                 // Base layer: Scrolling rainbow with bitwise hue shift
//                 LEDPattern rainbowBase = (reader, writer) -> {
//                     int bufLen = reader.getLength();
//                     double time = Timer.getFPGATimestamp();
//                     int shift = (int) (time * 20) % 32; // Fast shift for dynamic effect
//                     for (int i = 0; i < bufLen; i++) {
//                         // Bitwise XOR for hue variation
//                         int hue = (((i << 2) ^ shift) & 0xFF) % 180; // Shift left, XOR with time, mask to 0-255, fit to 0-180
//                         writer.setHSV(i, hue, 255, 128); // Full saturation, half brightness
//                     }
//                 };
//                 LEDPattern scrollingRainbow = rainbowBase.scrollAtRelativeSpeed(Percent.per(Second).of(50));

//                 // Middle layer: Breathing pattern with bitwise brightness control
//                 LEDPattern breathingLayer = (reader, writer) -> {
//                     double time = Timer.getFPGATimestamp();
//                     long periodMicros = (long) Second.of(1.5).in(Microseconds); // 1.5s breathing cycle
//                     double t = (RobotController.getTime() % periodMicros) / (double) periodMicros;
//                     double dim = (Math.cos(t * 2 * Math.PI) + 1) / 2.0; // Cosine fade from 0 to 1

//                     for (int i = 0; i < reader.getLength(); i++) {
//                         // Use bitwise AND to apply breathing only to every 4th LED
//                         if ((i & 3) == 0) {
//                             int r = (int) (reader.getRed(i) * dim);
//                             int g = (int) (reader.getGreen(i) * dim);
//                             int b = (int) (reader.getBlue(i) * dim);
//                             writer.setRGB(i, r, g, b);
//                         } else {
//                             writer.setRGB(i, reader.getRed(i), reader.getGreen(i), reader.getBlue(i));
//                         }
//                     }
//                 };

//                 // Top layer: Twinkling mask with bitwise randomness
//                 LEDPattern twinkleMask = (reader, writer) -> {
//                     double time = Timer.getFPGATimestamp();
//                     int timeSeed = (int) (time * 30) % 64; // Fast twinkle cycle
//                     for (int i = 0; i < reader.getLength(); i++) {
//                         // Bitwise OR and threshold for twinkling
//                         if (((i & 0xF) | timeSeed) > 50) {
//                             writer.setLED(i, Color.kWhite); // Twinkle on
//                         } else {
//                             writer.setLED(i, Color.kBlack); // Twinkle off (show base pattern)
//                         }
//                     }
//                 };

//                 // Combine layers: Rainbow scrolls, breathing pulses, twinkling overlays
//                 LEDPattern cosmicPattern = scrollingRainbow
//                     .breathe(Second.of(1.5)) // Add global breathing effect
//                     .overlayOn(breathingLayer) // Add selective breathing
//                     .mask(twinkleMask) // Apply twinkling mask
//                     .mapIndex((bufLen, index) -> {
//                         // Zigzag effect: Alternate direction every 10 LEDs
//                         int segment = index / 10;
//                         int localIndex = index % 10;
//                         return (segment & 1) == 0 ? index : (segment * 10 + (9 - localIndex));
//                     })
//                     .atBrightness(Percent.of(80)); // Slightly dim for visual comfort

//                 // Apply to buffer and update LED strip
//                 cosmicPattern.applyTo(cosmic_buffer); // Using policeLed_buffer as a spare
//                 ledBar.setData(cosmic_buffer);
//             }


//             // public static void setPoliceLights() {
//                 // // Get current time
//                 // double currentTime = Timer.getFPGATimestamp();

//                 // // Switch phases if enough time has passed
//                 // if (currentTime - policeLastChangeTime >= POLICE_BLINK_INTERVAL) {
//                 //     policeIsFirstPhase = !policeIsFirstPhase;
//                 //     policeLastChangeTime = currentTime;
//                 // }

//                 // // Set alternating patterns
//                 // for (int i = 0; i < LEDConstants.ledLength; i++) {
//                 //     if (policeIsFirstPhase) {
//                 //         // First half red, second half blue
//                 //         if (i < HALF_LENGTH) {
//                 //             policeLed_buffer.setLED(i, Color.kRed);
//                 //         } else {
//                 //             policeLed_buffer.setLED(i, Color.kBlue);
//                 //         }
//                 //     } else {
//                 //         // First half blue, second half red
//                 //         if (i < HALF_LENGTH) {
//                 //             policeLed_buffer.setLED(i, Color.kBlue);
//                 //         } else {
//                 //             policeLed_buffer.setLED(i, Color.kRed);
//                 //         }
//                 //     }
//                 // }

//                 // // Update LED strip
//                 // ledBar.setData(policeLed_buffer);
//             // }
        
//             // public static void elevatorMsg(){
//             //     LEDPattern elevatorGradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kTurquoise, Color.kNavy);
//             //     LEDPattern elevatoPattern = LEDPattern.progressMaskLayer(() -> elevator.getPosition()/ElevatorConstants.kElevatorMaxHeightRaw).overlayOn(elevatorGradient);
//             //     elevatoPattern.applyTo(elevator_progress_buffer);
//             //     ledBar.setData(elevator_progress_buffer);
//             // }

//     // public static void setRainbow(){ // sets rainbow led
//     //     // For every pixel
//     //     for (var i = 0; i < led_dynamic_msg.getLength(); i++) {
//     //       // Calculate the hue - hue is easier for rainbows because the color
//     //       // shape is a circle so only one value needs to precess
//     //       final var hue = (rainbowFirstPixelHue + (i * 180 / led_dynamic_msg.getLength())) % 180;
//     //       // Set the value
//     //       led_dynamic_msg.setHSV(i, hue, 255, 128);
//     //     }
//     //     // Increase by to make the rainbow "move"
//     //     rainbowFirstPixelHue += 3;
//     //     // Check bounds
//     //     rainbowFirstPixelHue %= 180;
//     // }

//     // public static void setDynamicMsg(){
//     //     if ( dynamicCount < 1000 ){
//     //         dynamicCount++;
//     //     }
//     //     setRainbow();
//     //     ledBar.setData(led_dynamic_msg);
//     // }
// }

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.TimerTask;

public class LedSubsystem extends SubsystemBase {

    // private static ElevatorSubsystem elevator = new ElevatorSubsystem();
            
    private static AddressableLED ledBar; // instance of the led bar class

    private static double allianceBlinkLastChangeTime = 0;
    private static boolean allianceBlinkIsOn = true;
    private static final double ALLIANCE_BLINK_INTERVAL_FAST = 0.3;
    private static final double ALLIANCE_BLINK_INTERVAL = 0.5;
    private static final double ALLIANCE_BLINK_INTERVAL_SLOW = 1.5;
    // Buffer or the holder of the color's data and its variable
    private static AddressableLEDBuffer led_red_alliance;
    private static AddressableLEDBuffer led_blue_alliance;
    private static AddressableLEDBuffer led_red_blue;
    private static AddressableLEDBuffer led_blank;
    private static AddressableLEDBuffer led_green;
    private static AddressableLEDBuffer led_yellow;
    private static AddressableLEDBuffer led_orange;
    private static AddressableLEDBuffer led_purple;
    // private static AddressableLEDBuffer led_dynamic_msg;
    private static AddressableLEDBuffer rainbow_buffer;
    private static AddressableLEDBuffer elevator_progress_buffer;
    private static AddressableLEDBuffer scroll_rainbow_buffer;
    private static AddressableLEDBuffer breath_buffer;
    private static AddressableLEDBuffer alliance_buffer;
    private static AddressableLEDBuffer autonomous_buffer;
    private static AddressableLEDBuffer policeLed_buffer;
    private static AddressableLEDBuffer blink_alliance_buffer;
    private static AddressableLEDBuffer checkerBoard_buffer;
    private static AddressableLEDBuffer counterChase_buffer;
    private static AddressableLEDBuffer cosmic_buffer;
    private static AddressableLEDBuffer scroll_buffer;

    private static LEDPattern scrollBase = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kCyan, Color.kTeal);
    private static LEDPattern allianceLED;
        
    private static double policeLastChangeTime = 0;
    private static boolean policeIsFirstPhase = true;
    private static final double POLICE_BLINK_INTERVAL = 0.05; // 50ms per phase (~6.67 Hz cycle)
    private static final int HALF_LENGTH = LEDConstants.ledLength / 2;
        
    // private static int dynamicCount = 0; // counter for dynamic messages
    private static int rainbowFirstPixelHue = 0; // tracks the hue for the first pixel in the rainbow effect
    // public static boolean dynamicEffect = false; // a tracking variable for checking dynamic effect
    private static java.util.Timer animationTimer = null; // Timer for self-sustaining animations
        
    // a block that runs once when class is initially loaded
    static {
        setAllianceColor();

        ledBar = new AddressableLED(LEDConstants.kLEDBarPWM); // initializes the led bar object with the given power port
        ledBar.setLength(LEDConstants.ledLength);
        
        // set buffers to their specified length
        
        led_red_alliance = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_blue_alliance = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_red_blue = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_blank = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        rainbow_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        elevator_progress_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        scroll_rainbow_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        breath_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        alliance_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_green = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_yellow = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_orange = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        autonomous_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        policeLed_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        blink_alliance_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        checkerBoard_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        counterChase_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        cosmic_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        led_purple = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
        scroll_buffer = new AddressableLEDBuffer(LEDConstants.ledLength);
        // led_dynamic_msg = new AddressableLEDBuffer(LEDConstants.ledBufferLength); // specific blinking message
        
        // initial message for led buffers
        for (int i = 0 ; i < LEDConstants.ledLength; i++){
            // pre-set all message buffers during initialization
            led_red_alliance.setLED(i, Color.kRed);
            led_blue_alliance.setLED(i, Color.kBlue);
            led_green.setLED(i, Color.kGreen);
            led_yellow.setLED(i, Color.kYellow);
            led_orange.setLED(i, Color.kOrange);
            led_purple.setLED(i, Color.kPurple);
        
            // using bitwise-AND operators .... (alternating red-blue led)
            if (((i & 3) == 0) // if the last two binary value of i & 3 is the same as 0's
             || ((i & 3) == 2) // if if the last two binary value of i & 3 is the same as 2's
             ){
                led_red_blue.setLED(i, Color.kRed);
            }  else {
                led_red_blue.setLED(i, Color.kBlue);
            }
        
            led_blank.setLED(i, Color.kBlack);
            // led_dynamic_msg.setLED(i, Color.kBlack); // no pre-set for dynamic msg
        }
        
        ledBar.setData(led_blank); // displays the black-led at first
        ledBar.start(); // activates the led strip
    }
        
    public static void stopLedBar(){
        ledBar.stop(); // turns off
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
    }
        
    public static void startLedBar(){
        ledBar.start(); // turns on
    }
        
    public static void setBlankMsg(){
        // dynamicEffect = false;
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        ledBar.setData(led_blank); // doesn't light up
    }
        
    public static void setRedBlueMsg(){
        // dynamicEffect = false; 
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        ledBar.setData(led_red_blue); // lights up red_blue
    }
        
    public static void setGreenMsg(){
        // dynamicEffect = false;
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        ledBar.setData(led_green); // lights up green
    }

    public static void setPurpleMsg(){
        // dynamicEffect = false;
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        ledBar.setData(led_purple); // lights up green
    }

    public static void setOrangeMsg(){
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        ledBar.setData(led_orange);
    }

    public static void setYellowMsg(){
        // dynamicEffect = false;
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        ledBar.setData(led_yellow); // lights up green
    }
        
    public static void setAllianceColor(){ // sets alliance color in led
        DriverStation.Alliance friendlyAlliance = DriverStation.getAlliance().get();
        if(friendlyAlliance == DriverStation.Alliance.Red){
            allianceLED = LEDPattern.solid(Color.kRed);
        }
        else if(friendlyAlliance == DriverStation.Alliance.Blue){
            allianceLED = LEDPattern.solid(Color.kBlue);
        }
        else {
            allianceLED = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
        }
    }

    public static Color getAllianceColor(){ // sets alliance color in led
        DriverStation.Alliance friendlyAlliance = DriverStation.getAlliance().get();
        if(friendlyAlliance == DriverStation.Alliance.Red){
            return Color.kRed;
        }
        else if(friendlyAlliance == DriverStation.Alliance.Blue){
            return Color.kBlue;
        }
        else {
            return Color.kPurple;
        }
    }

    public static void blinkAllianceSolid() {
        // Stop any existing animation
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        
        // Start a new timer for the blinking alliance color
        animationTimer = new java.util.Timer();
        animationTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                double currentTime = Timer.getFPGATimestamp();
                
                // Switch phases if enough time has passed
                if (currentTime - allianceBlinkLastChangeTime >= ALLIANCE_BLINK_INTERVAL) {
                    allianceBlinkIsOn = !allianceBlinkIsOn;
                    allianceBlinkLastChangeTime = currentTime;
                }
                
                if (allianceBlinkIsOn) {
                    // Set solid alliance color
                    allianceLED.applyTo(alliance_buffer);
                    ledBar.setData(alliance_buffer);
                } else {
                    // Turn off (black)
                    ledBar.setData(led_blank);
                }
            }
        }, 0, 20); // Start immediately, update every 20ms (50 Hz)
    }

    public static void blinkAllianceSolidFast() {
        // Stop any existing animation
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        
        // Start a new timer for the blinking alliance color
        animationTimer = new java.util.Timer();
        animationTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                double currentTime = Timer.getFPGATimestamp();
                
                // Switch phases if enough time has passed
                if (currentTime - allianceBlinkLastChangeTime >= ALLIANCE_BLINK_INTERVAL_FAST) {
                    allianceBlinkIsOn = !allianceBlinkIsOn;
                    allianceBlinkLastChangeTime = currentTime;
                }
                
                if (allianceBlinkIsOn) {
                    // Set solid alliance color
                    allianceLED.applyTo(alliance_buffer);
                    ledBar.setData(alliance_buffer);
                } else {
                    // Turn off (black)
                    ledBar.setData(led_blank);
                }
            }
        }, 0, 20); // Start immediately, update every 20ms (50 Hz)
    }

    public static void breathAllianceSolid(){
        // Stop any existing animation
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        
        // Start a new timer for the breathing effect
        animationTimer = new java.util.Timer();
        animationTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                // Get current time for animation
                double time = Timer.getFPGATimestamp();
                double period = 2.0;
                // Calculate brightness (0 to 0.5, matching 50% max brightness)
                double brightness = (Math.cos(time * 2 * Math.PI / period) + 1) / 4.0; // 0 to 0.5
                // Get alliance color
                Color allianceColor = getAllianceColor(); // Fallback for gradient case
                
                // Apply breathing effect to each pixel
                for (int i = 0; i < breath_buffer.getLength(); i++) {
                    int r = (int)(allianceColor.red * 255 * brightness);
                    int g = (int)(allianceColor.green * 255 * brightness);
                    int b = (int)(allianceColor.blue * 255 * brightness);
                    breath_buffer.setRGB(i, r, g, b);
                }
                
                // Update LED strip
                ledBar.setData(breath_buffer);
            }
        }, 0, 20); // Start immediately, update every 20ms (50 Hz)
    }

    public static void blinkAllianceSolidSlow() {
        // Stop any existing animation
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        
        // Start a new timer for the slow blinking alliance color
        animationTimer = new java.util.Timer();
        animationTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                double currentTime = Timer.getFPGATimestamp();
                
                // Switch phases if enough time has passed
                if (currentTime - allianceBlinkLastChangeTime >= ALLIANCE_BLINK_INTERVAL_SLOW) {
                    allianceBlinkIsOn = !allianceBlinkIsOn;
                    allianceBlinkLastChangeTime = currentTime;
                }
                
                if (allianceBlinkIsOn) {
                    // Set solid alliance color
                    allianceLED.applyTo(blink_alliance_buffer);
                    ledBar.setData(blink_alliance_buffer);
                } else {
                    // Turn off (black)
                    ledBar.setData(led_blank);
                }
            }
        }, 0, 20); // Start immediately, update every 20ms (50 Hz)
    }

    // public static void setAllianceBlink() {
    //     if (animationTimer != null) {
    //         animationTimer.cancel();
    //         animationTimer = null;
    //     }
    //     // Use allianceLED and apply a fast blink (0.2s cycle = 5 Hz)
    //     LEDPattern blinkPattern = allianceLED.blink(Second.of(0.2)).atBrightness(Percent.of(100));
                
    //     // Apply to the blink buffer and update LED strip
    //     blinkPattern.applyTo(blink_alliance_buffer);
    //     ledBar.setData(blink_alliance_buffer);
    // }

    public static void setAllianceSolid(){
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        allianceLED.applyTo(alliance_buffer);
        ledBar.setData(alliance_buffer);
    }

    private static final double SCROLL_INTERVAL = 0.005; // Time per scroll step in seconds
    private static double scrollLastChangeTime;
    private static int scrollPosition;
    private static int blockSize = 1;

    public static void setScrollingMsg() {
        // Stop any existing animation
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        
        // Start a new timer for the scrolling color
        animationTimer = new java.util.Timer();
        scrollPosition = 0;
        
        animationTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                double currentTime = Timer.getFPGATimestamp();

                // Advance scroll position if enough time has passed
                if (currentTime - scrollLastChangeTime >= SCROLL_INTERVAL) {
                    scrollPosition = (scrollPosition + 1) % LEDConstants.ledLength;
                    scrollLastChangeTime = currentTime;
                }

                // Set LED pattern: green block at current position, black elsewhere
                for (int i = 0; i < LEDConstants.ledLength; i++) {
                    if (i >= scrollPosition && i < scrollPosition + blockSize && i < LEDConstants.ledLength) {
                        scroll_buffer.setLED(i, Color.kDarkViolet);
                    } else {
                        scroll_buffer.setLED(i, Color.kBlack);
                    }
                }

                // Update LED strip
                ledBar.setData(scroll_buffer);
            }
        }, 0, 20);
    }
        
    public static void setScrollingRainbow(){
        // Stop any existing animation
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        
        // Start a new timer for the scrolling rainbow
        animationTimer = new java.util.Timer();
        animationTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                // Get current time for animation
                double time = Timer.getFPGATimestamp();
                // Speed factor: adjust this to control scrolling speed (higher = faster)
                double speed = 90.0; // 90 hue units per second (tuned for visible scrolling)
                
                // For every pixel
                for (int i = 0; i < scroll_rainbow_buffer.getLength(); i++) {
                    // Calculate the hue: base it on position and time for scrolling effect
                    final int hue = (int)((time * speed + i * 180.0 / scroll_rainbow_buffer.getLength()) % 180);
                    // Set the value
                    scroll_rainbow_buffer.setHSV(i, hue, 255, 128);
                }
                
                // Update LED strip
                ledBar.setData(scroll_rainbow_buffer);
            }
        }, 0, 20); // Start immediately, update every 20ms (50 Hz)
    }
        
    public static void setBreathingMsg(){
        // Stop any existing animation
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        
        // Start a new timer for the breathing effect
        animationTimer = new java.util.Timer();
        animationTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                // Get current time for animation
                double time = Timer.getFPGATimestamp();
                double period = 2.0;
                // Calculate brightness (0 to 0.5, matching 50% max brightness)
                double brightness = (Math.cos(time * 2 * Math.PI / period) + 1) / 4.0; // 0 to 0.5
                // Get alliance color
                Color allianceColor = allianceLED == LEDPattern.solid(Color.kRed) ? Color.kRed :
                                     allianceLED == LEDPattern.solid(Color.kBlue) ? Color.kBlue :
                                     Color.kOrangeRed; // Fallback for gradient case
                
                // Apply breathing effect to each pixel
                for (int i = 0; i < breath_buffer.getLength(); i++) {
                    int r = (int)(allianceColor.red * 255 * brightness);
                    int g = (int)(allianceColor.green * 255 * brightness);
                    int b = (int)(allianceColor.blue * 255 * brightness);
                    breath_buffer.setRGB(i, r, g, b);
                }
                
                // Update LED strip
                ledBar.setData(breath_buffer);
            }
        }, 0, 20); // Start immediately, update every 20ms (50 Hz)
    }
        
    public static void setRainbow(){
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        LEDPattern rainbow = LEDPattern.rainbow(255, 128).atBrightness(Percent.of(100));
        rainbow.applyTo(rainbow_buffer);
        ledBar.setData(rainbow_buffer);
    }

    public static void setPoliceLights() {
        // Stop any existing animation
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        
        // Start a new timer for the police lights
        animationTimer = new java.util.Timer();
        animationTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                double currentTime = Timer.getFPGATimestamp();

                // Switch phases if enough time has passed
                if (currentTime - policeLastChangeTime >= POLICE_BLINK_INTERVAL) {
                    policeIsFirstPhase = !policeIsFirstPhase;
                    policeLastChangeTime = currentTime;
                }

                // Set alternating patterns for each half
                for (int i = 0; i < LEDConstants.ledLength; i++) {
                    if (policeIsFirstPhase) {
                        // First phase: First half red, second half blue
                        if (i < HALF_LENGTH) {
                            policeLed_buffer.setLED(i, Color.kRed);
                        } else {
                            policeLed_buffer.setLED(i, Color.kBlue);
                        }
                    } else {
                        // Second phase: First half blue, second half red
                        if (i < HALF_LENGTH) {
                            policeLed_buffer.setLED(i, Color.kBlue);
                        } else {
                            policeLed_buffer.setLED(i, Color.kRed);
                        }
                    }
                }

                // Update LED strip
                ledBar.setData(policeLed_buffer);
            }
        }, 0, 20); // Start immediately, update every 20ms (50 Hz)
    }

    public static void setCosmicBinaryStorm() {
        // Stop any existing animation
        if (animationTimer != null) {
            animationTimer.cancel();
            animationTimer = null;
        }
        
        // Define the cosmic pattern (same as original)
        LEDPattern rainbowBase = (reader, writer) -> {
            int bufLen = reader.getLength();
            double time = Timer.getFPGATimestamp();
            int shift = (int) (time * 20) % 32; // Fast shift for dynamic effect
            for (int i = 0; i < bufLen; i++) {
                // Bitwise XOR for hue variation
                int hue = (((i << 2) ^ shift) & 0xFF) % 180; // Shift left, XOR with time, mask to 0-255, fit to 0-180
                writer.setHSV(i, hue, 255, 128); // Full saturation, half brightness
            }
        };
        LEDPattern scrollingRainbow = rainbowBase.scrollAtRelativeSpeed(Percent.per(Second).of(50));

        LEDPattern breathingLayer = (reader, writer) -> {
            double time = Timer.getFPGATimestamp();
            long periodMicros = (long) Second.of(1.5).in(Microseconds); // 1.5s breathing cycle
            double t = (RobotController.getTime() % periodMicros) / (double) periodMicros;
            double dim = (Math.cos(t * 2 * Math.PI) + 1) / 2.0; // Cosine fade from 0 to 1

            for (int i = 0; i < reader.getLength(); i++) {
                // Use bitwise AND to apply breathing only to every 4th LED
                if (i % 4 == 0) {
                    int r = (int) (reader.getRed(i) * dim);
                    int g = (int) (reader.getGreen(i) * dim);
                    int b = (int) (reader.getBlue(i) * dim);
                    writer.setRGB(i, r, g, b);
                } else {
                    writer.setRGB(i, reader.getRed(i), reader.getGreen(i), reader.getBlue(i));
                }
            }
        };

        LEDPattern twinkleMask = (reader, writer) -> {
            double time = Timer.getFPGATimestamp();
            int timeSeed = (int) (time * 30) % 64; // Fast twinkle cycle
            for (int i = 0; i < reader.getLength(); i++) {
                // Bitwise OR and threshold for twinkling
                if (((i & 0xF) | timeSeed) > 50) {
                    writer.setLED(i, Color.kWhite); // Twinkle on
                } else {
                    writer.setLED(i, Color.kBlack); // Twinkle off (show base pattern)
                }
            }
        };

        LEDPattern cosmicPattern = scrollingRainbow
            .breathe(Second.of(1.5)) // Add global breathing effect
            .overlayOn(breathingLayer) // Add selective breathing
            .mask(twinkleMask) // Apply twinkling mask
            .mapIndex((bufLen, index) -> {
                // Zigzag effect: Alternate direction every 10 LEDs
                int segment = index / 10;
                int localIndex = index % 10;
                return (segment & 1) == 0 ? index : (segment * 10 + (9 - localIndex));
            })
            .atBrightness(Percent.of(80)); // Slightly dim for visual comfort

        // Start a new timer for the cosmic pattern
        animationTimer = new java.util.Timer();
        animationTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                // Apply the cosmic pattern to the buffer
                cosmicPattern.applyTo(cosmic_buffer);
                // Update LED strip
                ledBar.setData(cosmic_buffer);
            }
        }, 0, 20); // Start immediately, update every 20ms (50 Hz)
    }
}