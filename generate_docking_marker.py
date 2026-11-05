"""Generate a printable, dimensioned ArUco PDF plus a PNG with white margin."""

import argparse
from pathlib import Path
import zlib

import cv2
import numpy as np


def write_pdf(path, image, side_metres):
    # A single grayscale image on a page with exact physical dimensions.
    height, width = image.shape
    points = side_metres / 0.0254 * 72
    compressed = zlib.compress(image.tobytes())
    content = f'q {points:.6f} 0 0 {points:.6f} 0 0 cm /Im0 Do Q'.encode()
    objects = [
        b'<< /Type /Catalog /Pages 2 0 R >>',
        b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
        (f'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {points:.6f} {points:.6f}] '
         '/Resources << /XObject << /Im0 4 0 R >> >> /Contents 5 0 R >>').encode(),
        (f'<< /Type /XObject /Subtype /Image /Width {width} /Height {height} '
         f'/ColorSpace /DeviceGray /BitsPerComponent 8 /Filter /FlateDecode '
         f'/Length {len(compressed)} >>\nstream\n').encode()+compressed+b'\nendstream',
        f'<< /Length {len(content)} >>\nstream\n'.encode()+content+b'\nendstream',
    ]
    data = bytearray(b'%PDF-1.4\n')
    offsets = [0]
    for i, obj in enumerate(objects, 1):
        offsets.append(len(data))
        data.extend(f'{i} 0 obj\n'.encode()+obj+b'\nendobj\n')
    start = len(data)
    data.extend(f'xref\n0 {len(offsets)}\n0000000000 65535 f \n'.encode())
    for offset in offsets[1:]:
        data.extend(f'{offset:010d} 00000 n \n'.encode())
    data.extend(f'trailer\n<< /Size {len(offsets)} /Root 1 0 R >>\nstartxref\n{start}\n%%EOF\n'.encode())
    path.write_bytes(data)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dictionary', default='DICT_4X4_50')
    parser.add_argument('--id', type=int, default=0)
    parser.add_argument('--size', type=float, default=0.12, help='Outer black square side, metres')
    parser.add_argument('--output', type=Path, default=Path('aruco_0_120mm.pdf'))
    options = parser.parse_args(args)
    if not np.isfinite(options.size) or options.size <= 0:
        parser.error('--size must be finite and positive')
    if not options.dictionary.startswith('DICT_') or not hasattr(cv2.aruco, options.dictionary):
        parser.error('Unknown dictionary')
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, options.dictionary))
    if not 0 <= options.id < len(dictionary.bytesList):
        parser.error('Marker ID outside dictionary')
    pixels = 900
    marker = (cv2.aruco.generateImageMarker(dictionary, options.id, pixels)
              if hasattr(cv2.aruco, 'generateImageMarker') else
              cv2.aruco.drawMarker(dictionary, options.id, pixels))
    margin = pixels // 6
    image = cv2.copyMakeBorder(marker, margin, margin, margin, margin,
                              cv2.BORDER_CONSTANT, value=255)
    options.output.parent.mkdir(parents=True, exist_ok=True)
    pdf_path = options.output.with_suffix('.pdf')
    png_path = options.output.with_suffix('.png')
    write_pdf(pdf_path, image, options.size*(pixels+2*margin)/pixels)
    cv2.imwrite(str(png_path), image)
    print(f'PDF: {pdf_path}\nPNG: {png_path}')
    print(f'{options.dictionary}, ID={options.id}, outer BLACK square = {options.size*1000:g} mm')
    print('Print PDF at actual size / 100%; measure the black square after printing.')


if __name__ == '__main__':
    main()
