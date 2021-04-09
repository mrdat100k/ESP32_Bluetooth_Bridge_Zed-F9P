#ifndef _PUBX_PARSE_H
#define _PUBX_PARSE_H

#include <Arduino.h>



class PUBX {
	public:

		/*Constructor*/
		PUBX(void* buffer, uint8_t len);

		/**
		 * @brief skip field which don't use
		* @param s pointer to first position of that field
		* @retval pointer to first position of the next field
		*/
		static const char* skipField (const char* s);

		/**
		 * @brief parse string to unsigned int number
		* @param s pointer to first position of that field
		* @param len length of that number you want to get from that field
		* @retval parsed field
		*/
		static unsigned int parseUnsignedInt(const char* s, uint8_t len);

		/**
		 * @brief parse string to float number 
		* @param s pointer to first position of that field
		* @param log10Multiplier the number of character you want in fraction
		* @param eptr pointer to the next field
		* @retval parsed field
		*/
		static long parseFloat(const char* s, uint8_t log10Multiplier,
							const char** eptr = nullptr);

		/**
		 * @brief parse longitude/latitude with degree minute format 
		* @param s pointer to first position of that field
		* @param degWidth width of degree
		* @param eptr pointer to the next field
		* @retval 
		*/
		static long parseDegreeMinute(const char* s, uint8_t degWidth,
									const char** eptr = nullptr);

		/**
		 * @brief get the field
		* @param s pointer to first position of that field
		* @param result field after parse
		* @retval location of the next field
		*/
		static const char* parseField(const char* s, char *result = nullptr,
									int len = 0);

		/**
		 * @brief generate CRC checksum
		* @param s pointer to first position of message
		* @param checksum checksum result
		* @retval pointer to checksum field of message
		*/							  
		static const char* generateChecksum(const char* s, char* checksum);

		/**
		 * @brief test checksum of message
		* @param s pointer to first position of message
		* @retval true when checksum is true
		*/
		static bool testChecksum(const char* s);


		/**
		 * @brief init value for all field
		* @param none
		* @retval node
		*/
		void clear (void);

		/**
		 * @brief set buffer for object
		* @param buf pointer to buffer
		* @param len length of buffer
		* @retval none
		*/
		void setBuffer (void* buf, uint8_t len);

		/****************************************************************/

		/*The following function is for user to get all value of message*/

		uint8_t getNumSatellite (void) const {
			return _numSvs;
		}

		long getLongitude (void) const
		{
			return _longitude;
		}

		long getLatitude (void) const
		{
			return _latitude;
		}

		uint32_t getHorizontalAccuracy (void) const
		{
			return _hAcc;
		}

		uint32_t getVerticalAccuracy (void) const
		{
			return _vAcc;
		}
		uint16_t getYear (void) const 
		{
			return _year;
		}
		uint8_t getMonth (void) const
		{
			return _month;
		}
		uint8_t getDay (void) const
		{
			return _day;
		}

		uint8_t getHour (void) const
		{
			return _hour;
		}

		uint8_t getSecond (void) const
		{
			return _second;
		}

		uint8_t getMinute (void) const
		{
			return _minute;
		}

		uint8_t getHundredth (void) const
		{
			return _hundredths;
		}

		long getAltitudeRef (void) const
		{
			return _altRef;
		}
		
		bool process (char c);

		bool processPOS (const char* s);

		long getAltitude (void) const
		{
			return _altitude;
		}

		/**
		* @brief Get the vertical dilution of precision (VDOP), in tenths
		* @details A VDOP value of 1.1 is returned as `11`
		* @return uint8_t
		*/
		uint8_t getVDOP(void) const 
		{
			return _vdop;
		}

		/**
		* @brief Get the position dilution of precision (PDOP), in tenths
		* @details A PDOP value of 1.1 is returned as `11`
		* @return uint8_t
		*/
		uint8_t getPDOP(void) const
		{
			return _pdop;
		}

		uint8_t getHDOP(void) const
		{
			return _hdop;
		}

		uint8_t getTDOP(void) const
		{
			return _tdop;
		}

		char getNavSystem(void) const
		{
			return _navSystem;
		}

		long getSpeed (void) const
		{
			return _speed;
		}

		long getCourse (void) const
		{
			return _course;
		}

		char getRTKStatus (void) const 
		{
			return (strcmp(_posMode,"A"));
		}

	protected:

		static inline bool isEndOfFields(char c)
		{
			return c == '*' || c == '\0' || c == '\r' || c == '\n';
		}

		const char* parseTime(const char* s);
		
		const char* parseDate(const char* s);

		bool processPUBX(const char* s);

		bool processGNS(const char* s);

		bool processGSA(const char* s);

		bool processGLL(const char* s);

		bool processGGA(const char *s);

		bool processRMC(const char* s);



	private:

		uint8_t _bufferLen;
		char* _buffer;
		char *_ptr;
		char _navSystem;
		char _navStatus[1];

		char _navStat[3];
		char _messageID[6];
		char _posMode[5];

		bool _isValid, _altitudeValid;
		long _latitude, _longitude, _altitude;
		long _altRef, _sep;
		long _course, _speed;
		long _vVel;
		long _diffAge;

		uint16_t _year;
		uint8_t _month, _day, _hour, _minute, _second, _hundredths;
		uint8_t _hdop;
		uint8_t _vdop;
		uint8_t _tdop;
		uint8_t _pdop;
		uint8_t _numSvs;
		uint32_t _hAcc;
		uint32_t _vAcc;


};

#endif