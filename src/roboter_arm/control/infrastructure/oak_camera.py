"""Optional DepthAI 3.6.1 adapter; USB/SDK access happens only in the camera worker."""
import importlib.metadata
from datetime import timedelta

from roboter_arm.control.domain.camera import CameraSample, DEPTH_PROFILE, DepthSample, PROFILE, capabilities
from roboter_arm.control.infrastructure.depth_images import encode_depth


class OakCamera:
    kind = 'oak'

    def __init__(self):
        self.device = self.pipeline = None
        self.frames = self.controls = None

    def open(self):
        import depthai as dai
        if importlib.metadata.version('depthai') != '3.6.1':
            raise RuntimeError('Install the pinned camera runtime (DepthAI 3.6.1)')
        self.dai = dai
        self.device = dai.Device()
        features = {c.socket: c for c in self.device.getConnectedCameraFeatures()}
        sockets = dai.CameraBoardSocket
        sensors = ((sockets.CAM_A, 'IMX214'), (sockets.CAM_B, 'OV7251'), (sockets.CAM_C, 'OV7251'))
        if self.device.getPlatform() != dai.Platform.RVC2 or any(
                socket not in features or features[socket].sensorName.upper() != sensor
                for socket, sensor in sensors):
            raise RuntimeError('Camera profile supports OAK-D Lite IMX214 / OV7251 stereo on RVC2 only')
        feature = features[sockets.CAM_A]
        self.pipeline = dai.Pipeline(self.device)
        self.pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
        camera = self.pipeline.create(dai.node.Camera).build(sockets.CAM_A)
        output = camera.requestOutput((PROFILE['width'], PROFILE['height']), type=dai.ImgFrame.Type.NV12,
            resizeMode=dai.ImgResizeMode.CROP, fps=PROFILE['fps'], enableUndistortion=PROFILE['undistortion'])
        mono = [self.pipeline.create(dai.node.Camera).build(socket).requestOutput(
                    (640, 480), type=dai.ImgFrame.Type.GRAY8, fps=PROFILE['fps'])
                for socket in (sockets.CAM_B, sockets.CAM_C)]
        stereo = self.pipeline.create(dai.node.StereoDepth).build(
            *mono, presetMode=dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setLeftRightCheck(True)
        stereo.initialConfig.setDepthUnit(dai.LengthUnit.MILLIMETER)
        stereo.setOutputSize(DEPTH_PROFILE['width'], DEPTH_PROFILE['height'])
        output.link(stereo.inputAlignTo)
        encoder = self.pipeline.create(dai.node.VideoEncoder).build(output, frameRate=PROFILE['fps'],
            profile=dai.VideoEncoderProperties.Profile.MJPEG, quality=PROFILE['jpeg_quality'])
        encoder.input.setBlocking(False)
        encoder.input.setMaxSize(1)
        sync = self.pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(milliseconds=DEPTH_PROFILE['sync_tolerance_ms']))
        sync.setSyncAttempts(-1)
        encoder.out.link(sync.inputs['rgb'])
        stereo.depth.link(sync.inputs['depth'])
        self.frames = sync.out.createOutputQueue(maxSize=1, blocking=False)
        self.controls = camera.inputControl.createInputQueue()
        self.pipeline.start()
        return capabilities(bool(feature.hasAutofocus), has_depth=True)

    def apply(self, changes):
        dai = self.dai
        control = dai.CameraControl()
        exposure = changes.get('exposure')
        if exposure:
            if exposure['mode'] == 'auto':
                control.setAutoExposureEnable()
            else:
                control.setManualExposure(exposure['time_us'], exposure['iso'])
        focus = changes.get('focus')
        if focus:
            if focus['mode'] == 'once':
                control.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
                control.setAutoFocusTrigger()
            else:
                control.setAutoFocusMode(dai.CameraControl.AutoFocusMode.OFF)
                control.setManualFocus(focus['lens_position'])
        balance = changes.get('white_balance')
        if balance:
            if balance['mode'] == 'auto':
                control.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
            else:
                control.setManualWhiteBalance(balance['temperature_k'])
        if 'anti_banding' in changes:
            name = {'off':'OFF', '50hz':'MAINS_50_HZ', '60hz':'MAINS_60_HZ', 'auto':'AUTO'}[changes['anti_banding']]
            control.setAntiBandingMode(getattr(dai.CameraControl.AntiBandingMode, name))
        self.controls.send(control)

    def read(self):
        if not self.pipeline.isRunning():
            raise RuntimeError('Camera pipeline stopped')
        group = self.frames.tryGet()
        if group is None:
            return None
        frame, depth = group['rgb'], group['depth']
        delta = abs(depth.getTimestampDevice()-frame.getTimestampDevice())
        if delta > timedelta(milliseconds=DEPTH_PROFILE['sync_tolerance_ms']):
            raise RuntimeError('Camera RGB/depth timestamps exceed the fixed synchronization tolerance')
        size = (PROFILE['width'], PROFILE['height'])
        if (frame.getWidth(), frame.getHeight()) != size or (depth.getWidth(), depth.getHeight()) != size:
            raise RuntimeError('Camera RGB/depth dimensions do not match the fixed aligned profile')
        pixels = depth.getFrame()
        if (depth.getType() != self.dai.ImgFrame.Type.RAW16 or pixels.dtype.kind != 'u'
                or pixels.dtype.itemsize != 2 or pixels.shape != (size[1], size[0])):
            raise RuntimeError('Camera depth output must be a two-dimensional unsigned 16-bit image')
        png, preview, valid_fraction = encode_depth(pixels.astype('<u2', copy=False).tobytes(),
            *size, DEPTH_PROFILE['preview_near_mm'], DEPTH_PROFILE['preview_far_mm'])
        jpeg = frame.getData().tobytes()
        lens = frame.getLensPosition()
        observed = dict(width=frame.getWidth(), height=frame.getHeight(),
                        time_us=round(frame.getExposureTime().total_seconds()*1000000),
                        iso=frame.getSensitivity(), lens_position=lens if lens >= 0 else None,
                        temperature_k=frame.getColorTemperature(), sensor_fps=frame.getFps(),
                        source_sequence=frame.getSequenceNum())
        depth_observed = dict(width=depth.getWidth(), height=depth.getHeight(), unit='mm',
                              invalid_value=0, aligned_to='rgb', source_sequence=depth.getSequenceNum(),
                              sync_delta_ms=round(delta.total_seconds()*1000, 3),
                              valid_fraction=valid_fraction)
        now = self.dai.Clock.now()
        return CameraSample(jpeg, observed, (now-frame.getTimestamp()).total_seconds(),
            DepthSample(png, preview, depth_observed, (now-depth.getTimestamp()).total_seconds()))

    def close(self):
        try:
            if self.pipeline is not None:
                try:
                    self.pipeline.stop()
                finally:
                    self.pipeline.wait()
        finally:
            try:
                if self.device is not None:
                    self.device.close()
            finally:
                self.frames = self.controls = self.pipeline = self.device = None
