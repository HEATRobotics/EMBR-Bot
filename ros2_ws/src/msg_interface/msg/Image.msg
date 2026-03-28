# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
# Header timestamp should be acquisition time of image, nested fields are built in
std_msgs/Header header
# Image height -> Number of Rows
uint32 height
# Image width -> Numher of Columns
uint32 width
# Encoding of pixes -- channel meaning, ordering, size
string encoding
# Is this Big-endian -> stores the most significant byte first 
uint8 is_bigendian
# Full row length in bytes
uint32 step
# Actual matrix data, size is (steps * rows)
uint8[] data