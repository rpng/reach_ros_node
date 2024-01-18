# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Patrick Geneva
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import re
import time
import calendar
import datetime
import math
import logging
logger = logging.getLogger('rosout')


def safe_float(field):
    try:
        return float(field)
    except ValueError:
        return float('NaN')


def safe_int(field):
    try:
        return int(field)
    except ValueError:
        return 0


def convert_latitude(field):
    return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0


def convert_longitude(field):
    return safe_float(field[0:3]) + safe_float(field[3:]) / 60.0


# convert the time into standard unix time in seconds
# https://stackoverflow.com/a/11111177
def convert_time(nmea_utc):
    # Get current time in UTC for date information
    utc_struct = time.gmtime()  # immutable, so cannot modify this one
    utc_list = list(utc_struct)
    # If one of the time fields is empty, return NaN seconds
    if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6] or not nmea_utc[7:]:
        return float('NaN')
    else:
        hours = safe_int(nmea_utc[0:2])
        minutes = safe_int(nmea_utc[2:4])
        seconds = safe_int(nmea_utc[4:6])
        milliseconds = safe_float(nmea_utc[7:])
        utc_list[3] = hours
        utc_list[4] = minutes
        utc_list[5] = seconds
        # Debug testing
        #print(milliseconds)
        #print(pow(10.0,len(str(milliseconds))))
        # Note we divide by the power that the sub-second has
        # 20 => 0.2 seconds
        # 100 =? 0.1 seconds
        # 02 => 0.02 seconds
        unix_time = calendar.timegm(tuple(utc_list)) + milliseconds/(10.0*len(str(milliseconds)))
        return unix_time


# Convert status flag to true=valid fix
def convert_status_flag(status_flag):
    if status_flag == "A":
        return True
    elif status_flag == "V":
        return False
    else:
        return False


# Convert from knotes into meters per second
def convert_knots_to_mps(knots):
    return safe_float(knots) * 0.514444444444


# Need this wrapper because math.radians doesn't auto convert inputs
def convert_deg_to_rads(degs):
    return math.radians(safe_float(degs))


"""
Format for this is a sentence identifier (e.g. "GGA") as the key, with a
tuple of tuples where each tuple is a field name, conversion function and index
into the split sentence
http://www.trimble.com/oem_receiverhelp/v4.44/en/NMEA-0183messages_MessageOverview.html
https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual1.pdf
"""
parse_maps = {
    "GGA": [
        ("utc_time", convert_time, 1),
        ("latitude", convert_latitude, 2),
        ("latitude_direction", str, 3),
        ("longitude", convert_longitude, 4),
        ("longitude_direction", str, 5),
        ("fix_type", int, 6),
        ("num_satellites", safe_int, 7),
        ("hdop", safe_float, 8),
        ("altitude", safe_float, 9),
        ("mean_sea_level", safe_float, 11),
        ],
    "GST": [
        ("utc_time", convert_time, 1),
        ("ellipse_sigma_major", safe_float, 3),
        ("ellipse_sigma_minor", safe_float, 4),
        ("ellipse_sigma_ori", safe_float, 5),
        ("latitude_sigma", safe_float, 6),
        ("longitude_sigma", safe_float, 7),
        ("altitude_sigma", safe_float, 8),
        ],
    "RMC": [
        ("utc_time", convert_time, 1),
        ("fix_valid", convert_status_flag, 2),
        ("latitude", convert_latitude, 3),
        ("latitude_direction", str, 4),
        ("longitude", convert_longitude, 5),
        ("longitude_direction", str, 6),
        ("speed", convert_knots_to_mps, 7),
        ("true_course", convert_deg_to_rads, 8),
        ],
    "VTG": [
        ("ori_true", convert_deg_to_rads, 1),
        ("ori_magnetic", convert_deg_to_rads, 3),
        ("speed", convert_knots_to_mps, 5),
        ],
    "GSV": [
        ("message_total", safe_int, 1),
        ("message_number", safe_int, 2),
        ("sat_in_view", safe_int, 3),
        ("sat_01_id", safe_int, 4),
        ("sat_01_elivation", convert_deg_to_rads, 5),
        ("sat_01_azimuth", convert_deg_to_rads, 6),
        ("sat_01_snr", safe_float, 7),
        ("sat_02_id", safe_int, 8),
        ("sat_02_elivation", convert_deg_to_rads, 9),
        ("sat_02_azimuth", convert_deg_to_rads, 10),
        ("sat_02_snr", safe_float, 11),
        ("sat_03_id", safe_int, 12),
        ("sat_03_elivation", convert_deg_to_rads, 13),
        ("sat_03_azimuth", convert_deg_to_rads, 14),
        ("sat_03_snr", safe_float, 15),
        ("sat_04_id", safe_int, 16),
        ("sat_04_elivation", convert_deg_to_rads, 17),
        ("sat_04_azimuth", convert_deg_to_rads, 18),
        ("sat_04_snr", safe_float, 19),
        ],
    "GSA": [
        ("mode", str, 1),
        ("fix_type", safe_int, 2),
        ("sat_id", safe_int, 3),
        ("pdop", safe_float, 4),
        ("hdop", safe_float, 5),
        ("vdop", safe_float, 6)
        ]
    }


def parse_nmea_sentence(nmea_sentence):
    # Check for a valid nmea sentence
    if not re.match('(^\$GP|^\$GA|^\$GN|^\$GL).*\*[0-9A-Fa-f]{2}$', nmea_sentence):
        logger.warn("Regex didn't match, sentence not valid NMEA? Sentence was: %s" % repr(nmea_sentence))
        return False

    # Remove the last bit after the asterisk, this is the checksum
    nmea_sentence = nmea_sentence.split("*",1)[0]
    
    # Split on the commas
    fields = [field.strip(',') for field in nmea_sentence.split(',')]

    # Ignore the $ and talker ID portions (e.g. GP, GL, GA, GN)
    sentence_type = fields[0][3:]

    # Check to see if we have a maping for this message type
    if not sentence_type in parse_maps:
        logger.warn("Sentence type %s not in parse map, ignoring." % repr(sentence_type))
        return False

    # Parse the message, and return it
    parse_map = parse_maps[sentence_type]
    parsed_sentence = {}
    for entry in parse_map:
        parsed_sentence[entry[0]] = entry[1](fields[entry[2]])
    return {sentence_type: parsed_sentence}
