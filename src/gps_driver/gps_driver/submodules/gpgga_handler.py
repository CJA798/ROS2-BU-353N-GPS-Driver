from utm import from_latlon
from builtin_interfaces.msg import Time


class GPGGAHandler():
    '''
    A class to handle GPGGA messages.
    '''
    def __init__(self) -> None:
        '''
        Initializes the GPGGAHandler class.
        '''
        pass

    def is_GPGGA(self, data: str) -> bool:
        '''
        Checks if the data is a GPGGA message.

        Args:
            data (str): The data to be checked.

        Returns:
            bool: True if the data is a GPGGA message, False otherwise.
        '''
        return 'GPGGA' in data
    
    def has_none(self, data: dict) -> bool:
        '''
        Checks if the data has None values.

        Args:
            data (dict): The data to be checked.

        Returns:
            bool: True if the data has None values, False otherwise.
        '''
        return None in data.values()

    def extract_data(self, data:str) -> dict:
        '''
        Extracts relevant data from a GPGGA message.

        Args:
            data (str): The GPGGA message.

        Returns:
            dict: The relevant data extracted from the GPGGA message.
        '''
        # Split data by comma
        GPGGA_data: list = data.split(',')
        
        # Extract relevant data
        relevant_data: dict = {
            'identifier': GPGGA_data[0],
            'UTC_time_stamp': GPGGA_data[1],
            'latitude_DM': float(GPGGA_data[2]),
            'latitude_direction': GPGGA_data[3],
            'longitude_DM': float(GPGGA_data[4]),
            'longitude_direction': GPGGA_data[5],
            'altitude': float(GPGGA_data[9]),
            'altitude_units': GPGGA_data[10],
        }

        # Check if relevant data has None values
        if self.has_none(relevant_data):
            return None

        return relevant_data

    def UTC_to_ROS2_Time(self, time_stamp:str) -> Time:
        '''
        Converts a UTC time stamp to a ROS2 Time message.

        Args:
            time_stamp (str): The UTC time stamp.

        Returns:
            time (Time): The ROS2 Time message.
        '''
        time = Time()
        time.sec = int(time_stamp[:2]) * 3600 + int(time_stamp[2:4]) * 60 + int(time_stamp[4:6])
        time.nanosec = int(time_stamp[7:])  * int(1e6) 
        
        return time

    def DD_lat_long_to_UTM(self, latitude: float, longitude: float) -> tuple:
        '''
        Converts latitude and longitude to UTM.

        Args:
            latitude (float): The latitude in decimal degrees.
            longitude (float): The longitude in decimal degrees.

        Returns:
            tuple: The UTM easting, northing, zone, and letter.

        Raises:
            Exception: If an error occurs.
        '''
        try:
            return from_latlon(latitude, longitude)
        
        except Exception as e:
            print(e)

            return None
    
    def DM_to_DD(self, value: float, direction: str) -> float:
        '''
        Converts degrees minutes to decimal degrees.

        Args:
            value (float): The value to be converted.
            direction (str): The direction of the value.

        Returns:
            float: The value in decimal degrees.
        '''
        # Convert value to string
        value: str = str(value).split('.')

        # Convert decimal minutes to degrees
        dec_min_in_deg: float = float(f'0.{value[1]}') / 60

        # Convert minutes to degrees
        minutes_in_deg: float = int(value[0][-2:]) / 60

        # Get degrees
        degrees: float = int(value[0][:-2]) + minutes_in_deg + dec_min_in_deg

        # Check direction
        if direction == 'S' or direction == 'W':
            return -degrees
        
        return degrees
