import math
from typing import Union, Dict

import utm


def latlon_to_utm(
    latitude: float,
    longitude: float,
    altitude: float = 0
) -> Dict[str, Union[float, int, str]]:
    try:
        utm_conversion = utm.from_latlon(latitude, longitude)
        easting = utm_conversion[0]
        northing = utm_conversion[1]
        zone_number = utm_conversion[2]
        band = utm_conversion[3]

        return {
            'easting': easting,
            'northing': northing,
            'altitude': altitude,
            'utm_zone': zone_number,
            'band': band
        }
    except Exception as e:
        raise Exception(f"Error during UTM conversion: {e}")


def utm_to_latlon(
    easting: float,
    northing: float,
    zone_number: int,
    zone_letter: str,
) -> Dict[str, float]:
    try:
        latlon = utm.to_latlon(
            easting, northing, zone_number, zone_letter=zone_letter)
        return {
            'latitude': latlon[0],
            'longitude': latlon[1]
        }
    except Exception as e:
        raise Exception(f"Error during UTM to LatLon conversion for \
            {easting}, {northing} [{zone_number, zone_letter}]: {e}")


def ned_to_enu(rad):
    # In NED, 0 rad = North, increasing clockwise
    # In ENU, 0 rad = East, increasing counter-clockwise
    enu = math.pi/2 - rad
    return (enu + 2 * math.pi) % (2 * math.pi)
