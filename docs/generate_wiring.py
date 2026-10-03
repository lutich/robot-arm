# /// script
# requires-python = ">=3.12"
# dependencies = ["pinviz==0.19.0"]
# ///
"""Regenerate the owner-confirmed wiring SVG: uv run docs/generate_wiring.py."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from pinviz import boards, Connection, Diagram, SVGRenderer
from pinviz.layout import CanvasSizer, LayoutConfig, LayoutResult, WireRouter
from pinviz.model import Device, DevicePin, PinRole, Point


def pin(name, role, x, y):
    return DevicePin(name, PinRole(role), Point(x, y))


logic = [('VCC (3.3 V)', '3V3'), ('SDA', 'I2C_SDA'),
         ('SCL', 'I2C_SCL'), ('GND', 'GND')]
pca_pins = [pin(name, role, 0, 55 + i * 28)
            for i, (name, role) in enumerate(logic)]
pca_pins += [pin('Servo V+', '5V', 0, 210), pin('Supply GND', 'GND', 0, 238)]
servos = []
connections = [Connection(number, 'PCA9685 / 0x40', name)
               for number, (name, _) in zip((1, 3, 5, 6), logic)]
for channel, joint in enumerate(('Base', 'Shoulder', 'Elbow', 'Wrist flex',
                                 'Wrist rotation', 'Gripper')):
    name = f'S{channel + 1} / {joint}'
    servos.append(Device(name, [pin('Signal', 'PWM', 0, 40),
                               pin('V+', '5V', 0, 60),
                               pin('GND', 'GND', 0, 80)],
                         width=180, height=95, color='#2563a6'))
    for offset, (label, role) in enumerate((('Signal', 'PWM'), ('V+', '5V'),
                                           ('GND', 'GND'))):
        source = f'CH{channel} {label}'
        pca_pins.append(pin(source, role, 250, 55 + channel * 110 + offset * 20))
        connections.append(Connection.from_device('PCA9685 / 0x40', source,
                                                   name, label))

supply = Device('Servo power supply', [pin('+', '5V', 190, 50),
                                  pin('-', 'GND', 190, 80)],
                width=190, height=115, color='#475569')
connections += [Connection.from_device(supply.name, '+', 'PCA9685 / 0x40',
                                       'Servo V+', color='#dc2626'),
                Connection.from_device(supply.name, '-', 'PCA9685 / 0x40',
                                       'Supply GND', color='#334155')]
pca = Device('PCA9685 / 0x40', pca_pins, width=250, height=670, color='#13795b')
diagram = Diagram('Robot arm — confirmed wiring', boards.raspberry_pi_4(),
                  [supply, pca, *servos], connections)
output = Path(__file__).parent / 'images' / 'wiring.svg'


class WiringLayout:
    """Keep corresponding channel and servo pins aligned for readable wiring."""

    def layout_diagram(self, diagram):
        config = LayoutConfig(board_margin_left=430)
        top = config.get_board_margin_top(diagram.show_title)
        supply.position = Point(450, 550)
        pca.position = Point(950, 145)
        for channel, servo in enumerate(servos):
            servo.position = Point(1460, 160 + channel * 110)
        wires = WireRouter(config, config.board_margin_left, top).route_wires(diagram)
        width, height = CanvasSizer(config, top).calculate_canvas_size(diagram, wires)
        return LayoutResult(width, height, Point(config.board_margin_left, top),
                            {device.name: device.position for device in diagram.devices},
                            wires, top)


renderer = SVGRenderer(LayoutConfig(board_margin_left=430))
renderer.layout_engine = WiringLayout()
renderer.render(diagram, output)

# Keep the generated diagram self-contained when opened outside the README.
ET.register_namespace('', 'http://www.w3.org/2000/svg')
tree = ET.parse(output)
root = tree.getroot()
namespace = '{http://www.w3.org/2000/svg}'
# Enlarge diagram labels without changing the small markings in board artwork.
for node in root.findall(namespace + 'text'):
    size = node.get('font-size')
    if size == '7':
        node.set('font-size', '20')
        node.set('y', str(float(node.get('y')) + 4))
        if node.text.startswith('CH'):
            node.set('x', str(pca.position.x + pca.width - 10))
            node.set('text-anchor', 'end')
        elif float(node.get('x')) > pca.position.x + pca.width:
            node.set('x', str(servos[0].position.x - 12))
            node.set('text-anchor', 'end')
            node.set('fill', '#334155')
            node.set('stroke', '#ffffff')
            node.set('stroke-width', '3')
            node.set('paint-order', 'stroke')
    elif size in ('12.0', '14'):
        node.set('font-size', '24')
        if node.text.startswith('S') and ' / ' in node.text:
            node.set('y', str(float(node.get('y')) + 7))
    elif size == '20':
        node.set('font-size', '30')
        node.set('y', '40')
artwork = ET.parse(output.with_name('wiring-components.svg')).getroot()
root.insert(0, deepcopy(artwork.find(namespace + 'defs')))
for device in diagram.devices:
    kind = 'bench-supply' if device is supply else 'pca-board' if device is pca else 'metal-servo'
    box = next(node for node in root if node.tag == namespace + 'rect'
               and node.get('x') == str(device.position.x)
               and node.get('y') == str(device.position.y)
               and node.get('fill') == device.color)
    # Replace only the body artwork; PinViz retains the connected pins and labels.
    root[list(root).index(box)] = ET.Element(namespace + 'use', href='#' + kind,
                                            x=str(device.position.x), y=str(device.position.y))
ET.SubElement(root, namespace + 'use', href='#mac-laptop', x='40', y='150')
marker = ET.SubElement(root.find(namespace + 'defs'), namespace + 'marker',
                      id='network-arrow', viewBox='0 0 10 10', refX='9', refY='5',
                      markerWidth='6', markerHeight='6', orient='auto-start-reverse')
ET.SubElement(marker, namespace + 'path', d='M0 0L10 5L0 10Z', fill='#2563a6')
ET.SubElement(root, namespace + 'path', d='M340 250H430', fill='none',
              stroke='#2563a6', **{'stroke-width': '2.5', 'stroke-dasharray': '6 4',
                                  'marker-start': 'url(#network-arrow)',
                                  'marker-end': 'url(#network-arrow)'})
for x, y, label in ((190, 360, 'Mac laptop'), (190, 392, 'macOS · Browser + uv tools'),
                    (385, 202, 'SSH'), (385, 230, 'HTTP'), (385, 112, 'Wi-Fi / Ethernet'),
                    (545, 690, '(Eventek KPS305D)')):
    ET.SubElement(root, namespace + 'text', x=str(x), y=str(y), fill='#334155',
                  style='font: 22px Arial, sans-serif', **{'text-anchor': 'middle'}).text = label
width, height = map(float, root.get('viewBox').split()[2:])
root.set('height', str(height + 110))
root.set('viewBox', f'0 0 {width} {height + 110}')
ET.SubElement(root, namespace + 'rect', x='0', y=str(height), width=str(width),
              height='110', fill='#ffffff')
for index, note in enumerate((
    'Pi header: physical 1 = 3.3 V, 3 = SDA / GPIO2, 5 = SCL / GPIO3, 6 = GND.',
    'Pi uses its official USB-C supply. Servo V+ uses the separate bench supply; all GND pins share ground.',
    'Wiring confirmed by the owner on 2026-10-04. Component artwork and pin placement are schematic.',
    'Generated with PinViz 0.19.0. Wire colors identify functions; they do not specify actual cable colors.',
)):
    ET.SubElement(root, namespace + 'text', x='40', y=str(height + 20 + index * 24),
                  fill='#334155', style='font: 22px Arial, sans-serif').text = note
ET.SubElement(root, namespace + 'title').text = diagram.title
ET.SubElement(root, namespace + 'desc').text = (
    'Owner-confirmed wiring. A Mac laptop connects to the Pi using SSH over Wi-Fi or Ethernet. '
    'Pi physical pins 1, 3, 5 and 6 feed PCA9685 logic VCC, SDA, SCL and GND. '
    'A separate servo power supply (Eventek KPS305D) feeds servo V+ and common GND. '
    'Channels 0–5 each connect signal, V+ and GND to servos S1–S6.')
tree.write(output, encoding='utf-8', xml_declaration=True)
