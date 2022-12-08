import struct
from sensor_msgs.msg import PointField

def get_fields():
    return [PointField('x', 0, PointField.FLOAT32, 1),
      PointField('y', 4, PointField.FLOAT32, 1),
      PointField('z', 8, PointField.FLOAT32, 1),
      PointField('rgb', 12, PointField.UINT32, 1),
     ]
def pack_colors(r,g,b,a):
    return struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
