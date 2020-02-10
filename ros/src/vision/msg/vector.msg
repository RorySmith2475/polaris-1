string      id                  # The ID of the detected object
uint8       type                # Category: 0=Gate, 1=Path
uint16      x                   # cm
uint16      y                   # cm
uint16      z                   # cm
float32     angle               # degrees (radian?)
bool        object_found        # {0,1}
uint8       accuracy            # 0 -> 100
# string      type                # which object is being detected