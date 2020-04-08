// 
// 
// 

#include "RX8900.h"


////////////////////////////////////////////////////////////////////////////////
// utility code, some of this could be exposed in the DateTime API if needed

const uint8_t daysInMonth[] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
	if (y >= 2000)
		y -= 2000;
	uint16_t days = d;
	for (uint8_t i = 1; i < m; ++i)
		days += pgm_read_byte(daysInMonth + i - 1);
	if (m > 2 && y % 4 == 0)
		++days;
	return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
	return ((days * 24L + h) * 60 + m) * 60 + s;
}

////////////////////////////////////////////////////////////////////////////////
// DateTime implementation - ignores time zones and DST changes
// NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second

DateTime::DateTime(uint32_t t) {
	t -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970

	ss = t % 60;
	t /= 60;
	mm = t % 60;
	t /= 60;
	hh = t % 24;
	uint16_t days = t / 24;
	uint8_t leap;
	for (yOff = 0; ; ++yOff) {
		leap = yOff % 4 == 0;
		if (days < 365 + leap)
			break;
		days -= 365 + leap;
	}
	for (m = 1; ; ++m) {
		uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
		if (leap && m == 2)
			++daysPerMonth;
		if (days < daysPerMonth)
			break;
		days -= daysPerMonth;
	}
	d = days + 1;
}

DateTime::DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
	if (year >= 2000)
		year -= 2000;
	yOff = year;
	m = month;
	d = day;
	hh = hour;
	mm = min;
	ss = sec;
}

DateTime::DateTime(const DateTime& copy) :
	yOff(copy.yOff),
	m(copy.m),
	d(copy.d),
	hh(copy.hh),
	mm(copy.mm),
	ss(copy.ss)
{}

static uint8_t conv2d(const char* p) {
	uint8_t v = 0;
	if ('0' <= *p && *p <= '9')
		v = *p - '0';
	return 10 * v + *++p - '0';
}

// A convenient constructor for using "the compiler's time":
//   DateTime now (__DATE__, __TIME__);
// NOTE: using F() would further reduce the RAM footprint, see below.
DateTime::DateTime(const char* date, const char* time) {
	// sample input: date = "Dec 26 2009", time = "12:34:56"
	yOff = conv2d(date + 9);
	// Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec 
	switch (date[0]) {
	case 'J': m = date[1] == 'a' ? 1 : m = date[2] == 'n' ? 6 : 7; break;
	case 'F': m = 2; break;
	case 'A': m = date[2] == 'r' ? 4 : 8; break;
	case 'M': m = date[2] == 'r' ? 3 : 5; break;
	case 'S': m = 9; break;
	case 'O': m = 10; break;
	case 'N': m = 11; break;
	case 'D': m = 12; break;
	}
	d = conv2d(date + 4);
	hh = conv2d(time);
	mm = conv2d(time + 3);
	ss = conv2d(time + 6);
}

// A convenient constructor for using "the compiler's time":
// This version will save RAM by using PROGMEM to store it by using the F macro.
//   DateTime now (F(__DATE__), F(__TIME__));
DateTime::DateTime(const __FlashStringHelper* date, const __FlashStringHelper* time) {
	// sample input: date = "Dec 26 2009", time = "12:34:56"
	char buff[11];
	memcpy_P(buff, date, 11);
	yOff = conv2d(buff + 9);
	// Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
	switch (buff[0]) {
	case 'J': m = buff[1] == 'a' ? 1 : m = buff[2] == 'n' ? 6 : 7; break;
	case 'F': m = 2; break;
	case 'A': m = buff[2] == 'r' ? 4 : 8; break;
	case 'M': m = buff[2] == 'r' ? 3 : 5; break;
	case 'S': m = 9; break;
	case 'O': m = 10; break;
	case 'N': m = 11; break;
	case 'D': m = 12; break;
	}
	d = conv2d(buff + 4);
	memcpy_P(buff, time, 8);
	hh = conv2d(buff);
	mm = conv2d(buff + 3);
	ss = conv2d(buff + 6);
}

uint8_t DateTime::dayOfTheWeek() const {
	uint16_t day = date2days(yOff, m, d);
	return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

uint32_t DateTime::unixtime(void) const {
	uint32_t t;
	uint16_t days = date2days(yOff, m, d);
	t = time2long(days, hh, mm, ss);
	t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000

	return t;
}

long DateTime::secondstime(void) const {
	long t;
	uint16_t days = date2days(yOff, m, d);
	t = time2long(days, hh, mm, ss);
	return t;
}

DateTime DateTime::operator+(const TimeSpan& span) {
	return DateTime(unixtime() + span.totalseconds());
}

DateTime DateTime::operator-(const TimeSpan& span) {
	return DateTime(unixtime() - span.totalseconds());
}

TimeSpan DateTime::operator-(const DateTime& right) {
	return TimeSpan(unixtime() - right.unixtime());
}

////////////////////////////////////////////////////////////////////////////////
// TimeSpan implementation

TimeSpan::TimeSpan(int32_t seconds) :
	_seconds(seconds)
{}

TimeSpan::TimeSpan(int16_t days, int8_t hours, int8_t minutes, int8_t seconds) :
	_seconds((int32_t)days * 86400L + (int32_t)hours * 3600 + (int32_t)minutes * 60 + seconds)
{}

TimeSpan::TimeSpan(const TimeSpan& copy) :
	_seconds(copy._seconds)
{}

TimeSpan TimeSpan::operator+(const TimeSpan& right) {
	return TimeSpan(_seconds + right._seconds);
}

TimeSpan TimeSpan::operator-(const TimeSpan& right) {
	return TimeSpan(_seconds - right._seconds);
}
////////////////////////////////////////////////////////////////////////////////
// RTC_Millis implementation

long RTC_Millis::offset = 0;

void RTC_Millis::adjust(const DateTime& dt) {
	offset = dt.unixtime() - millis() / 1000;
}

DateTime RTC_Millis::now() {
	return (uint32_t)(offset + millis() / 1000);
}

////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// RTC_RX8900 implementation

static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }


boolean RTC_RX8900::begin(void) {
	
	return true;
}

boolean RTC_RX8900::initialized(void) {

	Wire0_beginTransmission(RX8900_ADDRESS);
	Wire0_write((byte)RX8900_FLAG);
	Wire0_endTransmission();
	Wire0_requestFrom(RX8900_ADDRESS, 1);
	uint8_t ss = Wire0_read();
	return ((ss & 0x02) != 0x00);			//return VoltageLowFlag bit
}

uint16_t RTC_RX8900::TimerINT_Second(uint16_t count) {

	//enter 11bit value for sycle time LSB  (MAX:4096seconds)
	Wire0_beginTransmission(RX8900_ADDRESS);
	Wire0_write((byte)RX8900_TIMER_COUNTER0);
	Wire0_write((byte)0x05);

	Wire0_endTransmission();
	//MSB
	Wire0_beginTransmission(RX8900_ADDRESS);
	Wire0_write((byte)RX8900_TIMER_COUNTER1);
	Wire0_write((byte)0x00);

	Wire0_endTransmission();
	//Enable timer , set second update
	Wire0_beginTransmission(RX8900_ADDRESS);
	Wire0_write((byte)RX8900_EXTENTION);
	Wire0_write((byte)0x12);
	Wire0_endTransmission();
	//set TImer Interroupt Enable
	Wire0_beginTransmission(RX8900_ADDRESS);
	Wire0_write((byte)RX8900_CONTROL);
	Wire0_write((byte)0x10);
	Wire0_endTransmission();
}

void RTC_RX8900::adjust(const DateTime& dt) {

	Wire0_beginTransmission(RX8900_ADDRESS);
	Wire0_write((byte)RX8900_EXTENTION);
	Wire0_write((byte)0x00);
	Wire0_endTransmission();

	Wire0_beginTransmission(RX8900_ADDRESS);
	Wire0_write((byte)RX8900_FLAG);
	Wire0_write((byte)0x00);
	Wire0_endTransmission();
	Wire0_beginTransmission(RX8900_ADDRESS);
	Wire0_write((byte)0); // start at location 3
	Wire0_write(bin2bcd(dt.second()));
	Wire0_write(bin2bcd(dt.minute()));
	Wire0_write(bin2bcd(dt.hour()));
	Wire0_write((0)); // skip weekdays
	Wire0_write(bin2bcd(dt.day()));
	Wire0_write(bin2bcd(dt.month()));
	Wire0_write(bin2bcd(dt.year() - 2000));
	Wire0_endTransmission();

	// set to battery switchover mode
	Wire0_beginTransmission(RX8900_ADDRESS);
	Wire0_write((byte)RX8900_BACKUP);
	Wire0_write((byte)0x04);
	Wire0_endTransmission();
}

DateTime RTC_RX8900::now() {
	Wire0_beginTransmission(RX8900_ADDRESS);
	Wire0_write((byte)0);
	Wire0_endTransmission();

	Wire0_requestFrom(RX8900_ADDRESS, 7);
	uint8_t ss = bcd2bin(Wire0_read() & 0x7F);
	uint8_t mm = bcd2bin(Wire0_read());
	uint8_t hh = bcd2bin(Wire0_read());
	Wire0_read();  // skip 'weekdays'
	uint8_t d = bcd2bin(Wire0_read());
	uint8_t m = bcd2bin(Wire0_read());
	uint8_t y = bcd2bin(Wire0_read()) - 18;

	return DateTime(y, m, d, hh, mm, ss);
}

