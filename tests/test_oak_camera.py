"""Pinned SDK graph and atomic RGB/depth pairing, without SDK import or a device."""
from datetime import timedelta
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.infrastructure.oak_camera import OakCamera


class OakCameraTests(unittest.TestCase):
    def sdk(self):
        cameras = [Mock() for _ in range(3)]
        for camera in cameras:
            camera.build.return_value = camera
        stereo, encoder, sync = Mock(), Mock(), Mock()
        stereo.build.return_value, encoder.build.return_value = stereo, encoder
        sync.inputs = dict(rgb=Mock(), depth=Mock())
        pipeline = Mock()
        pipeline.create.side_effect = [*cameras, stereo, encoder, sync]
        device = Mock()
        device.getPlatform.return_value = 'RVC2'
        device.getConnectedCameraFeatures.return_value = [
            SimpleNamespace(socket=socket, sensorName=sensor, hasAutofocus=af)
            for socket, sensor, af in (('A', 'IMX214', True), ('B', 'OV7251', False), ('C', 'OV7251', False))]
        sdk = SimpleNamespace(Device=Mock(return_value=device), Pipeline=Mock(return_value=pipeline),
            CameraBoardSocket=SimpleNamespace(CAM_A='A', CAM_B='B', CAM_C='C'),
            Platform=SimpleNamespace(RVC2='RVC2'), ImgFrame=SimpleNamespace(Type=SimpleNamespace(
                NV12='NV12', GRAY8='GRAY8', RAW16='RAW16')),
            ImgResizeMode=SimpleNamespace(CROP='CROP'), LengthUnit=SimpleNamespace(MILLIMETER='mm'),
            VideoEncoderProperties=SimpleNamespace(Profile=SimpleNamespace(MJPEG='MJPEG')),
            node=SimpleNamespace(Camera=object(), VideoEncoder=object(), Sync=object(),
                StereoDepth=SimpleNamespace(PresetMode=SimpleNamespace(DEFAULT='DEFAULT'))))
        sdk.Pipeline.AutoCalibrationMode = SimpleNamespace(OFF='OFF')
        return sdk, device, pipeline, cameras, stereo, encoder, sync

    def test_profile_graph_uses_real_rgb_geometry_and_one_strict_paired_output(self):
        sdk, device, pipeline, cameras, stereo, encoder, sync = self.sdk()
        source = OakCamera()
        with patch.dict(sys.modules, depthai=sdk), patch('importlib.metadata.version', return_value='3.6.1'):
            caps = source.open()
        self.assertTrue(caps['has_depth'])
        self.assertTrue(caps['has_autofocus'])
        for camera, socket in zip(cameras, ('A', 'B', 'C')):
            camera.build.assert_called_once_with(socket)
        cameras[0].requestOutput.assert_called_once_with((1280, 720), type='NV12',
            resizeMode='CROP', fps=5, enableUndistortion=True)
        for camera in cameras[1:]:
            camera.requestOutput.assert_called_once_with((640, 480), type='GRAY8', fps=5)
        rgb, left, right = [camera.requestOutput.return_value for camera in cameras]
        stereo.build.assert_called_once_with(left, right, presetMode='DEFAULT')
        stereo.setLeftRightCheck.assert_called_once_with(True)
        stereo.initialConfig.setDepthUnit.assert_called_once_with('mm')
        stereo.setOutputSize.assert_called_once_with(1280, 720)
        rgb.link.assert_called_once_with(stereo.inputAlignTo)
        encoder.build.assert_called_once_with(rgb, frameRate=5, profile='MJPEG', quality=85)
        encoder.input.setBlocking.assert_called_once_with(False)
        encoder.input.setMaxSize.assert_called_once_with(1)
        sync.setSyncThreshold.assert_called_once_with(timedelta(milliseconds=20))
        sync.setSyncAttempts.assert_called_once_with(-1)
        encoder.out.link.assert_called_once_with(sync.inputs['rgb'])
        stereo.depth.link.assert_called_once_with(sync.inputs['depth'])
        encoder.out.createOutputQueue.assert_not_called()
        sync.out.createOutputQueue.assert_called_once_with(maxSize=1, blocking=False)
        pipeline.start.assert_called_once_with()
        pipeline.setAutoCalibrationMode.assert_called_once_with('OFF')
        source.close()
        self.assertIsNone(source.frames)
        self.assertIsNone(source.controls)

    def test_profile_refuses_missing_stereo_or_other_platform(self):
        for invalid in ('missing_right', 'platform'):
            sdk, device, pipeline, *_ = self.sdk()
            if invalid == 'missing_right':
                device.getConnectedCameraFeatures.return_value.pop()
            else:
                device.getPlatform.return_value = 'RVC4'
            source = OakCamera()
            with patch.dict(sys.modules, depthai=sdk), patch('importlib.metadata.version', return_value='3.6.1'):
                with self.subTest(invalid=invalid), self.assertRaisesRegex(RuntimeError, 'OAK-D Lite'):
                    source.open()
            pipeline.start.assert_not_called()
            source.close()
            device.close.assert_called_once_with()

    def frames(self):
        source = OakCamera()
        rgb, depth = Mock(), Mock()
        for frame in (rgb, depth):
            frame.getWidth.return_value, frame.getHeight.return_value = 1280, 720
            frame.getTimestampDevice.return_value = timedelta(seconds=1)
            frame.getTimestamp.return_value = timedelta(seconds=10)
        rgb.getExposureTime.return_value = timedelta(microseconds=8000)
        rgb.getSensitivity.return_value, rgb.getLensPosition.return_value = 200, 130
        rgb.getColorTemperature.return_value, rgb.getFps.return_value = 4500, 15
        rgb.getSequenceNum.return_value, depth.getSequenceNum.return_value = 20, 19
        rgb.getData.return_value.tobytes.return_value = b'jpeg'
        depth.getType.return_value = 'RAW16'
        depth.getTimestampDevice.return_value += timedelta(milliseconds=10)
        depth.getTimestamp.return_value -= timedelta(milliseconds=10)
        pixels = Mock(shape=(720, 1280), dtype=SimpleNamespace(kind='u', itemsize=2))
        pixels.astype.return_value.tobytes.return_value = b'\1\0'
        depth.getFrame.return_value = pixels
        source.dai = SimpleNamespace(ImgFrame=SimpleNamespace(Type=SimpleNamespace(RAW16='RAW16')),
                                     Clock=Mock())
        source.pipeline, source.frames = Mock(), Mock()
        source.pipeline.isRunning.return_value = True
        source.frames.tryGet.return_value = dict(rgb=rgb, depth=depth)
        return source, rgb, depth, pixels

    def test_atomic_pair_metadata_and_age_include_encoding_time(self):
        source, rgb, depth, pixels = self.frames()
        source.dai.Clock.now.return_value = timedelta(seconds=10.02)
        def encode(*args):
            source.dai.Clock.now.return_value = timedelta(seconds=10.5)
            return b'metric', b'preview', .75
        with patch('roboter_arm.control.infrastructure.oak_camera.encode_depth', side_effect=encode) as encode_mock:
            sample = source.read()
        encode_mock.assert_called_once_with(b'\1\0', 1280, 720, 200, 3000)
        pixels.astype.assert_called_once_with('<u2', copy=False)
        self.assertEqual((sample.jpeg, sample.depth.png, sample.depth.preview_png), (b'jpeg', b'metric', b'preview'))
        self.assertEqual((sample.age, sample.depth.age), (.5, .51))
        self.assertEqual(sample.observed['source_sequence'], 20)
        self.assertEqual(sample.observed['sensor_fps'], 15)
        self.assertEqual(sample.depth.observed, dict(width=1280, height=720, unit='mm', invalid_value=0,
            aligned_to='rgb', source_sequence=19, sync_delta_ms=10.0, valid_fraction=.75))

    def test_pair_refuses_skew_geometry_and_non_metric_dtype(self):
        for invalid in ('skew', 'width', 'signed', 'type'):
            source, rgb, depth, pixels = self.frames()
            if invalid == 'skew':
                depth.getTimestampDevice.return_value += timedelta(milliseconds=11)
            elif invalid == 'width':
                depth.getWidth.return_value = 640
            elif invalid == 'signed':
                pixels.dtype.kind = 'i'
            else:
                depth.getType.return_value = 'GRAY8'
            with patch('roboter_arm.control.infrastructure.oak_camera.encode_depth') as encode:
                with self.subTest(invalid=invalid), self.assertRaises(RuntimeError):
                    source.read()
                encode.assert_not_called()

    def test_empty_output_does_not_make_a_partial_sample(self):
        source, *_ = self.frames()
        source.frames.tryGet.return_value = None
        self.assertIsNone(source.read())
        source.pipeline.isRunning.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'pipeline stopped'):
            source.read()


if __name__ == '__main__':
    unittest.main()
