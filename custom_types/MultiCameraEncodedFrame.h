

#ifndef MULTI_CAMERA_ENCODEDFRAME_H
#define MULTI_CAMERA_ENCODEDFRAME_H

#include <draco/core/encoder_buffer.h>
#include <vector>
#include <map>
#include "../FrameDataType.h"
#include "../EncodedFrame.h"

using namespace std;

struct MultiCameraEncodedFrame {

  FrameDataType type;
  long frame_id;
  std::map<int, shared_ptr<EncodedFrame>> frame_data_per_camera;
  };
#endif //MULTI_CAMERA_ENCODEDFRAME_H
