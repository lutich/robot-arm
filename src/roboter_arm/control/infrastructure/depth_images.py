"""Lossless metric PNG and fixed-scale preview; no OpenCV or image dependency."""
from functools import lru_cache
import struct
import zlib


def _chunk(kind, payload):
    return struct.pack('>I', len(payload)) + kind + payload + struct.pack('>I', zlib.crc32(kind+payload))


def _png(pixels, width, height, bit_depth, color_type):
    stride = width * (3 if color_type == 2 else 1) * (bit_depth//8)
    if width <= 0 or height <= 0 or len(pixels) != stride*height:
        raise ValueError('Image dimensions must match the pixel buffer')
    rows = b''.join(b'\0'+pixels[start:start+stride] for start in range(0, len(pixels), stride))
    header = struct.pack('>IIBBBBB', width, height, bit_depth, color_type, 0, 0, 0)
    return (b'\x89PNG\r\n\x1a\n' + _chunk(b'IHDR', header)
            + _chunk(b'IDAT', zlib.compress(rows, level=1)) + _chunk(b'IEND', b''))


def metric_png(pixels_u16le, width, height):
    """Preserve every little-endian uint16 value, including zero, in PNG's big endian form."""
    if len(pixels_u16le) != width*height*2:
        raise ValueError('Image dimensions must match the uint16 pixel buffer')
    pixels = bytearray(len(pixels_u16le))
    pixels[0::2], pixels[1::2] = pixels_u16le[1::2], pixels_u16le[0::2]
    return _png(pixels, width, height, 16, 0)


@lru_cache(maxsize=1)
def colour_table(near_mm, far_mm):
    """RGB lookup: zero black; distances clamp from near red to far blue."""
    if not 0 < near_mm < far_mm:
        raise ValueError('Preview limits must be positive and increasing')
    colors = bytearray(65536*3)
    for distance in range(1, 65536):
        blue = max(0, min(255, (distance-near_mm)*255//(far_mm-near_mm)))
        colors[distance*3:distance*3+3] = bytes((255-blue, 0, blue))
    return bytes(colors)


def encode_depth(pixels_u16le, width, height, near_mm, far_mm):
    """NumPy is already part of the optional camera runtime; import it only for a frame."""
    import numpy as np
    metric = metric_png(pixels_u16le, width, height)
    depth = np.frombuffer(pixels_u16le, dtype='<u2').reshape(height, width)
    colors = np.frombuffer(colour_table(near_mm, far_mm), dtype=np.uint8).reshape(65536, 3)
    preview = _png(colors[depth].tobytes(), width, height, 8, 2)
    return metric, preview, float(np.count_nonzero(depth))/depth.size
