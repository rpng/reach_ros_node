# Positioning Solution Status. See page 101 in doc/rtklib_manaual_2.4.2.pdf.
Header header
string date
string utc_time
float64 latitude
float64 longitude
float64 altitude

# Quality of our fix.
uint8 QUALITY_FLAG_FIX = 1
uint8 QUALITY_FLAG_FLOAT = 2
uint8 QUALITY_FLAG_RESERVED = 3
uint8 QUALITY_FLAG_DGPS = 4
uint8 QUALITY_FLAG_SINGLE = 5
uint8 quality_flag

# Number of valid satellites.
uint8 num_sats

float64[9] position_covariance
uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3
uint8 position_covariance_type

# RTKLIB-specific stats.
float64 diff_age
float64 ratio
