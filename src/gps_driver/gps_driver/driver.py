import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from custom_interfaces.msg import GPSmsg

from utm import from_latlon

from .submodules.serial_com_handler import SerialHandler
from .submodules.gpgga_handler import GPGGAHandler


class GNSSDriver(Node):
    '''
    This class is used to create a ROS2 node that publishes GPS messages.
    
    Attributes:
        sh (SerialHandler): Serial handler used to read data from the GPS receiver.
        gpgga (GPGGAHandler): GPGGA handler used to extract data from GPGGA messages.
        timer (rclpy.timer.Timer): Timer used to read data from the serial port.
        gps_publisher (rclpy.publisher.Publisher): Publisher used to publish GPS messages
    
    Parameters:
        port (str): Serial port used to communicate with the GPS receiver.
        baudrate (int): Baudrate used to communicate with the GPS receiver.
    '''
    def __init__(self) -> None:
        '''
        This method is used to initialize the GNSSDriver class.
        '''
        # Initialize the Node class
        super().__init__('gps_publisher')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 4800)

        # Create a publisher
        self.gps_publisher = self.create_publisher(GPSmsg, 'gps', 10)

        # Create a serial handler
        self.get_logger().info(f"Opening serial port {self.get_parameter('port').value} with baudrate {self.get_parameter('baudrate').value}.")
        self.sh = SerialHandler(self.get_parameter('port').value, self.get_parameter('baudrate').value)
        
        # Check if serial port was succesfully open
        if self.sh.ser is None:
            self.get_logger().error('Failed to open serial port.')
            raise Exception('Failed to open serial port.')
        
        # Create a GPGGA handler
        self.gpgga = GPGGAHandler()
        self.get_logger().info('GPGGAHandler initialized.')

        # Create a timer to read data from the serial port
        self.timer = self.create_timer(1/20, self.timer_callback)


    def timer_callback(self) -> None:
        '''
        This method publishes a GPS message.
        '''
        # Read data from serial port
        data: str = self.sh.read_line()
        self.get_logger().info(f'Read: {data}')

        # Check if data is a GPGGA string
        if not self.gpgga.is_GPGGA(data):
            self.get_logger().warning('Ignored non-GPGGA data...')
            return
        
        # Extract data from GPGGA string
        self.get_logger().info('Extracting data...')
        data: dict = self.gpgga.extract_data(data)
        
        # Check if data is valid, i.e. no null values
        if data is None:
            self.get_logger().warning('Ignoring invalid data (None field(s))...')
            return

        # Convert UTC time to ROS2 Time
        self.get_logger().info('Converting UTC time to ROS2 Time...')
        time_stamp = self.gpgga.UTC_to_ROS2_Time(data['UTC_time_stamp'])

        # Convert latitude and longitude to decimal degrees
        self.get_logger().info('Converting latitude and longitude to decimal degrees...')
        latitude = self.gpgga.DM_to_DD(data['latitude_DM'], data['latitude_direction'])
        longitude = self.gpgga.DM_to_DD(data['longitude_DM'], data['longitude_direction'])

        # Convert latitude and longitude to UTM
        self.get_logger().info('Converting latitude and longitude to UTM...')
        utm_easting, utm_northing, utm_zone, utm_letter = self.gpgga.DD_lat_long_to_UTM(latitude, longitude)

        # Create message
        msg = GPSmsg()
        msg.header = Header(frame_id="GPS1_Frame", stamp=time_stamp)
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = data['altitude']
        msg.utm_easting = utm_easting
        msg.utm_northing = utm_northing
        msg.zone = utm_zone
        msg.letter = utm_letter
        
        # Publish message
        self.gps_publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')
        self.get_logger().info(f'Port: {self.get_parameter("port").value}')



def main(args=None):
    # Initialize ROS2 node
    try:
        rclpy.init(args=args)

        gnss_driver = GNSSDriver()

        rclpy.spin(gnss_driver)

    # Handle exceptions
    except Exception as e:
        print(e)
        gnss_driver.sh.close_port()
        gnss_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
