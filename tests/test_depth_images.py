"""PNG byte tests preserve uint16 metric values without optional camera dependencies."""
from pathlib import Path
import importlib.util
import struct
import sys
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.infrastructure.depth_images import _png, colour_table, encode_depth, metric_png


class DepthImageTests(unittest.TestCase):
    def decode(self, png):
        self.assertEqual(png[:8], b'\x89PNG\r\n\x1a\n')
        offset, chunks = 8, {}
        while offset < len(png):
            length = struct.unpack('>I', png[offset:offset+4])[0]
            kind, payload = png[offset+4:offset+8], png[offset+8:offset+8+length]
            checksum = struct.unpack('>I', png[offset+8+length:offset+12+length])[0]
            self.assertEqual(checksum, zlib.crc32(kind+payload))
            chunks[kind] = payload
            offset += length+12
        self.assertEqual(offset, len(png))
        self.assertEqual(chunks[b'IEND'], b'')
        return struct.unpack('>IIBBBBB', chunks[b'IHDR']), zlib.decompress(chunks[b'IDAT'])

    def test_metric_png_preserves_zero_full_range_and_byte_order(self):
        values = (0, 1, 255, 256, 3000, 65535)
        header, rows = self.decode(metric_png(struct.pack('<6H', *values), 3, 2))
        self.assertEqual(header, (3, 2, 16, 0, 0, 0, 0))
        self.assertEqual(rows, b'\0'+struct.pack('>3H', *values[:3])
                               +b'\0'+struct.pack('>3H', *values[3:]))

    def test_metric_png_refuses_invalid_buffer_or_dimensions(self):
        for pixels, width, height in ((b'\0', 1, 1), (b'\0\0', 2, 1), (b'', 0, 1)):
            with self.subTest(width=width, height=height), self.assertRaises(ValueError):
                metric_png(pixels, width, height)

    def test_fixed_preview_clips_only_colors_and_marks_unknown_black(self):
        table = colour_table(200, 3000)
        self.assertEqual(len(table), 65536*3)
        for distance, rgb in ((0, b'\0\0\0'), (1, b'\xff\0\0'), (200, b'\xff\0\0'),
                              (1600, b'\x80\0\x7f'), (3000, b'\0\0\xff'),
                              (65535, b'\0\0\xff')):
            self.assertEqual(table[distance*3:distance*3+3], rgb)
        header, rows = self.decode(_png(table[0:3]+table[600:603]+table[9000:9003], 3, 1, 8, 2))
        self.assertEqual(header, (3, 1, 8, 2, 0, 0, 0))
        self.assertEqual(rows, b'\0\0\0\0\xff\0\0\0\0\xff')

    def test_preview_limits_must_be_positive_and_increasing(self):
        for near, far in ((0, 3000), (3000, 200), (200, 200)):
            with self.subTest(near=near, far=far), self.assertRaises(ValueError):
                colour_table(near, far)

    @unittest.skipUnless(importlib.util.find_spec('numpy'), 'NumPy belongs to the optional camera runtime')
    def test_vectorized_preview_and_valid_fraction(self):
        metric, preview, fraction = encode_depth(struct.pack('<4H', 0, 200, 1600, 65535), 2, 2, 200, 3000)
        self.assertEqual(fraction, .75)
        self.assertEqual(self.decode(metric)[1], b'\0'+struct.pack('>2H', 0, 200)
                                                +b'\0'+struct.pack('>2H', 1600, 65535))
        self.assertEqual(self.decode(preview)[1], b'\0\0\0\0\xff\0\0\0\x80\0\x7f\0\0\xff')


if __name__ == '__main__':
    unittest.main()
