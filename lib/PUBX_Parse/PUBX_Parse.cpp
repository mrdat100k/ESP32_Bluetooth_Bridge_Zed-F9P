#include "PUBX_Parse.h"




long exp10(uint8_t b)
{
	long r = 1;
	while (b--)
		r *= 10;
	return r;
}


char toHex(uint8_t nibble)
{
	if (nibble >= 10)
		return nibble + 'A' - 10;
	else
		return nibble + '0';
}


const char* PUBX::skipField(const char* s)
{
	if (s == nullptr)
		return nullptr;

	while (!isEndOfFields(*s)) {
		if (*s == ',') {
			// Check next character
			if (isEndOfFields(*++s))
				break;
			else
				return s;
		}
		++s;
	}
	return nullptr;
}


unsigned int PUBX::parseUnsignedInt(const char *s, uint8_t len)
{
	int r = 0;
	while (len--)
		r = 10 * r + *s++ - '0';
	return r;
}


long PUBX::parseFloat(const char* s, uint8_t log10Multiplier, const char** eptr)
{
	int8_t neg = 1;
	long r = 0;
	while (isspace(*s))
		++s;
	if (*s == '-') {
		neg = -1;
		++s;
	}
	else if (*s == '+')
		++s;

	while (isdigit(*s))
		r = 10*r + *s++ - '0';
	r *= exp10(log10Multiplier);

	if (*s == '.')
	{
		++s;
		long frac = 0;
		while (isdigit(*s) && log10Multiplier)
		{
			frac = 10 * frac + *s++ -'0';
			--log10Multiplier;
		}
		frac *= exp10(log10Multiplier);
		r += frac;
	}
	r *= neg; // Include effect of any minus sign

	if (eptr)
		*eptr = skipField(s);

	return r;
}


long PUBX::parseDegreeMinute(const char* s, uint8_t degWidth,
								  const char **eptr)
{
	if (*s == ',') {
		if (eptr)
			*eptr = skipField(s);
		return 0;
	}
	long r = parseUnsignedInt(s, degWidth) * 1000000L;
	s += degWidth;
	r += parseFloat(s, 6, eptr) / 60;
	return r;
}

const char* PUBX::parseField(const char* s, char *result, int len)
{
	if (s == nullptr)
		return nullptr;

	int i = 0;
	while (*s != ',' && !isEndOfFields(*s)) {
		if (result && i++ < len)
			*result++ = *s;
		++s;
	}
	if (result && i < len)
		*result = '\0'; // Terminate unless too long

	if (*s == ',')
		return ++s; // Location of start of next field
	else
		return nullptr; // End of string or valid sentence
}


const char* PUBX::generateChecksum(const char* s, char* checksum)
{
	uint8_t c = 0;
	// Initial $ is omitted from checksum, if present ignore it.
	if (*s == '$')
		++s;

	while (*s != '\0' && *s != '*')
		c ^= *s++;

	if (checksum) {
		checksum[0] = toHex(c / 16);
		checksum[1] = toHex(c % 16);
	}
	return s;
}


bool PUBX::testChecksum(const char* s)
{
	char checksum[2];
	const char* p = generateChecksum(s, checksum);
	return (*p == '*' && p[1] == checksum[0] && p[2] == checksum[1]);
}


void PUBX::setBuffer (void* buf, uint8_t len)
{
    _bufferLen = len;
    _buffer = (char*)buf;
    _ptr = _buffer;
    if (_bufferLen)
    {
        *_ptr = '\0';
        _buffer[_bufferLen - 1]= '\0';
    }
}


void PUBX::clear (void)
{	
	_navSystem = '\0';
	_numSvs = 0;
	_hdop = 255;
	_vdop = 255;
	_tdop = 255;
	_isValid = 0;
	_altitudeValid = 0;
	_latitude = 999000000L;
	_longitude = 999000000L;
	_altitude = _speed = _course = _altRef = LONG_MIN;
	_hour = _minute = _second = 99;
	_year = _month = _day = _hundredths = 0;
	_hAcc = 9999;
	_vAcc = 9999;
}


PUBX::PUBX (void* buf, uint8_t len)
{
    setBuffer (buf,len);
    clear ();
}


bool PUBX::process (char c)
{
    if ((_buffer == nullptr) || (_bufferLen == 0))
        return false;
    if ((c == '\0') || (c == '\n') || (c=='\r')) 
	{
        *_ptr = '\0';
        _ptr = _buffer;
		
        if ((*_buffer == '$') && (testChecksum(_buffer)) )
		{
			const char* data;
			//Serial.println(_buffer);
			if (_buffer[1] == 'P')
			{
				data = parseField (&_buffer[6], &_messageID[0], sizeof(_messageID));
				if ((data != nullptr) && (strcmp(&_messageID[0],"00") == 0))
				{
					return processPOS (data);
				}
			}
			else if (_buffer[1] == 'G')
			{
				_navSystem = _buffer[2];
				data = parseField(&_buffer[3], &_messageID[0], sizeof(_messageID));

				if (data != nullptr && strcmp(&_messageID[0], "GGA") == 0)
				{}
					//return processGGA(data);
				else if (data != nullptr && strcmp(&_messageID[0], "RMC") == 0)
					return processRMC(data);
				else if (data != nullptr && strcmp(&_messageID[0], "GLL") == 0)
				{}
					//return processGLL(data);
				else if (data != nullptr && strcmp(&_messageID[0], "GSA") == 0)
				{}
					//return processGSA(data);
				else if (data != nullptr && strcmp(&_messageID[0], "GNS") == 0)
				{}
					//return processGNS(data);
			}
		}
        return *_buffer != '\0';
    }
    else
    {
        *_ptr = c;
        if (_ptr < &_buffer [_bufferLen - 1])
            ++_ptr;
    }

    return false;
}


const char* PUBX::parseTime(const char* s)
{
	if (*s == ',')
		return skipField(s);
	_hour = parseUnsignedInt(s, 2);
	_minute = parseUnsignedInt(s + 2, 2);
	_second = parseUnsignedInt(s + 4, 2);
	_hundredths = parseUnsignedInt(s + 7, 2);
	return skipField(s + 9);
}


const char* PUBX::parseDate(const char* s)
{
	if (*s == ',')
		return skipField(s);
	_day = parseUnsignedInt(s, 2);
	_month = parseUnsignedInt(s + 2, 2);
	_year = parseUnsignedInt(s + 4, 2) + 2000;
	return skipField(s + 6);
}


bool PUBX::processPOS(const char* s)
{
	s = parseTime (s);
	if (s == nullptr)
		return false;

	_latitude = parseDegreeMinute(s, 2, &s);
	if (s == nullptr)
		return false;

	if (*s == ',')		//NS
		++s;
	else
	{
		if (*s == 'S')
			_latitude *= -1;
		s += 2;
	}

	_longitude = parseDegreeMinute (s, 3, &s);
	if (s == nullptr)
		return false;

	if (*s == ',')		//EW
		++s;
	else {
		if (*s == 'W')
			_longitude *= -1;
		s += 2;
	}
	
	_altRef = parseFloat(s, 3, &s);
	if (s == nullptr)
		return false;

	s = parseField(s, _navStat, sizeof(_navStat));
	
	_hAcc = parseFloat (s, 2, &s);
	if (s == nullptr)
		return false;
	
	_vAcc = parseFloat (s, 2, &s);
	if (s == nullptr)
		return false;
	
	_speed = parseFloat(s, 3, &s);
	if (s == nullptr)
		return false;

	_course = parseFloat(s, 2, &s);
	if (s == nullptr)
		return false;

	_vVel = parseFloat(s, 3, &s);
	if (s == nullptr)
		return false;

	_diffAge = parseFloat(s, 1, &s);
	if (s == nullptr)
		return false;

	_hdop = parseFloat(s, 2, &s);
	if (s == nullptr)
		return false;

	_vdop = parseFloat(s, 2, &s);
	if (s == nullptr)
		return false;

	_tdop = parseFloat(s, 2, &s);
	if (s == nullptr)
		return false;

	_numSvs = parseFloat(s, 0, &s);
	if (s == nullptr)
		return false;


	return true;
}


bool PUBX::processGNS(const char* s)
{

	s = parseTime(s);
	if (s == nullptr)
		return false;

	_latitude = parseDegreeMinute(s, 2, &s);
	if (s == nullptr)
		return false;

	if (*s == ',')		//NE
		++s;
	else
	{
		if (*s == 'S')
			_latitude *= -1;
		s += 2; // Skip (N or S) and comma
	}

	_longitude = parseDegreeMinute(s, 3, &s);
	if (s == nullptr)
		return false;
	
	if (*s == ',')		//EW
		++s;
	else
	{
		if (*s == 'W')
			_longitude *= -1;
		s += 2; // Skip (E or W) and comma
	}

	s = parseField(s, _posMode, sizeof(_posMode));
	if (s == nullptr)
		return false;

	_numSvs = parseFloat(s, 0, &s);
	if (s == nullptr)
		return false;
	
	_hdop = parseFloat(s, 1, &s);
	if (s == nullptr)
		return false;
	
	_altitude = parseFloat(s, 3, &s);
	if (s == nullptr)
		return false;	

	_sep = parseFloat(s, 1, &s);
	//that is all we r interested in

	return true;
}


bool PUBX::processGSA(const char* s)
{
	int i = 0;
	while(i < 14)
	{
		s = skipField(s);
		if (s == nullptr)
			return false;
		i++;
	}

	_pdop = parseFloat(s, 2, &s);
	if (s == nullptr)
		return false;

	_hdop = parseFloat(s, 2, &s);
	if (s == nullptr)
		return false;

	_vdop = parseFloat(s, 2, &s);
	if (s == nullptr)
		return false;

	
	//that is all we r interested in	

	return true;
}


bool PUBX::processGLL(const char* s)
{

	_latitude = parseDegreeMinute(s, 2, &s);
	if (s == nullptr)
		return false;

	if (*s == ',')		 //NS
		++s;
	else
	{
		if (*s == 'S')
			_latitude *= -1;
		s += 2; // Skip (N or S) and comma
	}

	_longitude = parseDegreeMinute(s, 3, &s);
	if (s == nullptr)
		return false;
	
	if (*s == ',')		//EW
		++s;
	else
	{
		if (*s == 'W')
			_longitude *= -1;
		s += 2; // Skip (E or W) and comma
	}

	s = parseTime(s);
	if (s == nullptr)
		return false;

	_isValid = (*s == 'A');
	s += 2;
	
	//that is all we care about here

	return true;
}


bool PUBX::processGGA(const char *s)
{

	s = parseTime(s);
	if (s == nullptr)
		return false;

	_latitude = parseDegreeMinute(s, 2, &s);
	if (s == nullptr)
		return false;

	if (*s == ',')
		++s;
	else {
		if (*s == 'S')
			_latitude *= -1;
		s += 2; // Skip N/S and comma
	}

	_longitude = parseDegreeMinute(s, 3, &s);
	if (s == nullptr)
		return false;

	if (*s == ',')
		++s;
	else {
		if (*s == 'W')
			_longitude *= -1;
		s += 2; // Skip E/W and comma
	}

	_isValid = (*s >= '1' && *s <= '5');
	s += 2; // Skip position fix flag and comma

	_numSvs = parseFloat(s, 0, &s);
	if (s == nullptr)
		return false;

	_hdop = parseFloat(s, 2, &s);
	if (s == nullptr)
		return false;

	_altitude = parseFloat(s, 3, &s);
	if (s == nullptr)
		return false;

	_altitudeValid = true;
	// That's all we care about
	return true;
}


bool PUBX::processRMC(const char* s)
{

	s = parseTime(s);
	if (s == nullptr)
		return false;

	_isValid = (*s == 'A');
	s += 2; 	// Skip validity and comma

	_latitude = parseDegreeMinute(s, 2, &s);
	if (s == nullptr)
		return false;

	if (*s == ',')		// Skip N/S and comma
		++s;
	else
	{
		if (*s == 'S')
			_latitude *= -1;
		s += 2;
	}

	_longitude = parseDegreeMinute(s, 3, &s);
	if (s == nullptr)
		return false;
	
	if (*s == ',')		// Skip E/W and comma
		++s;
	else {
		if (*s == 'W')
			_longitude *= -1;
		s += 2;
	}

	//_speed = parseFloat(s, 3, &s);
	s = skipField(s);
	if (s == nullptr)
		return false;

	_course = parseFloat(s, 3, &s);
	if (s == nullptr)
		return false;

	s = parseDate(s);
	if (s == nullptr)
		return false;

	s = skipField(s);
	if (s == nullptr)
		return false;

	s = skipField(s);
	if (s == nullptr)
		return false;

	s = parseField(s, _posMode, 1);
	if (s == nullptr)
		return false;

	s = parseField(s, _navStatus, 1);
	if (s == nullptr)
		return false;

	// That's all we care about


	return true;
}

