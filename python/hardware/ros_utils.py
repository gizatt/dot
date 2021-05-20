
from std_msgs.msg import (
    Int16MultiArray, MultiArrayLayout, MultiArrayDimension
)

def convert_np_vector_to_int16_multi_array(vector):
    assert len(vector.shape) == 1
    N = vector.shape[0]
    msg = Int16MultiArray(
        layout=MultiArrayLayout(
            dim=[MultiArrayDimension(
                label="",
                size=N,
                stride=1
            )],
            data_offset=0
        ),
        data=vector.astype(int).tolist())
    return msg