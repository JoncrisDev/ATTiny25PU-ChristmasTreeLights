
// defines for LED pins
#define LED0 PB0
#define LED1 PB1
#define BUTTON0 PB4
// BUTTON PB0

// BOOL as uint8_t
#define TRUE							1
#define FALSE							0
#define BOOL							uint8_t

// As Our GPIO is configured with a pull- up resistance a circuit to gnd is a logical 0 and open is logical 1.
#define BUTTON_DOWN						FALSE
#define BUTTON_UP						TRUE

// Values that can affect an LEDLine behaviour
struct LEDLine {
	BOOL bEnabled;
	BOOL bIsFade;
	BOOL bIsHigh;
	BOOL bFadingUp;
	BOOL bFadingDown;
	uint8_t Pin;
	uint16_t TimeFadeUp;
	uint16_t TimeFadeDown;
	uint16_t TimeIsOn;
	uint16_t TimeIsOff;
	uint16_t ElapsedTimeInState;
};

// Holder for the different modes the controller can be in.
struct LEDMode {
	struct LEDLine line_1;
	struct LEDLine line_2;
};

// LED on off toggles
#define LED_On(pin) (PORTB |= (1 << pin))
#define LED_Off(pin) (PORTB &= ~(1 << pin))

// Get the Logical pin input value from a given input pin
//#define GetPinLevel(pin) return ( (PINB & (1 << pin)) > 0 ) ? TRUE : FALSE)
static inline BOOL GetPinLevel(uint8_t pin) { return ( (PINB & (1 << pin)) > 0 ) ? TRUE : FALSE;  }

// Turn the Lights and controller off after a long button hold
void RunMode(struct LEDMode * mode);
void RunLine(struct LEDLine * line);
void SwapOutMode(struct LEDMode * mode);
void ResetLine(struct LEDLine * Line);
void MCU_PowerDown(void);

// Program Control features
const uint8_t NUM_MODES = 3;
const uint8_t HOLD_TIME_PROGRAM_CHANGE = 50;
const uint16_t HOLD_TIME_POWER_DWON = 3000;
uint8_t CurrentModeID = 0;

// Current Pulse Length and rate at which to increase the pulse as we cycle levels through the push button
BOOL ButtonPrevState;