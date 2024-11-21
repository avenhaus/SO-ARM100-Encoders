#!/usr/bin/env python3

# Copyright (c) 2022-23, Bradley A. Minch
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met: 
# 
#     1. Redistributions of source code must retain the above copyright 
#        notice, this list of conditions and the following disclaimer. 
#     2. Redistributions in binary form must reproduce the above copyright 
#        notice, this list of conditions and the following disclaimer in the 
#        documentation and/or other materials provided with the distribution. 
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE.

import math
import shlex
import sys
from enum import Enum

class Point:

    x = None
    y = None

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f'({self.x}, {self.y})'

    def __repr__(self):
        return f'({self.x}, {self.y})'

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        return Point(x, y)

    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        return self

    def __sub__(self, other):
        x = self.x - other.x
        y = self.y - other.y
        return Point(x, y)

    def __isub__(self, other):
        self.x -= other.x
        self.y -= other.y

    def mag(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

def parse_svg_file(svg_filename):

    class States(Enum):
        SCANNING = 1
        IN_SVG = 2
        IN_PATH = 3
        IN_CIRCLE = 4

    svg_file = open(svg_filename, 'r')

    svg_properties = {}
    paths = []

    state = States.SCANNING
    for line in svg_file:
        line = line.strip()
        while len(line) != 0:
            if state == States.SCANNING:
                if '<svg' in line:
                    line = line[line.find('<svg') + 4:]
                    element = ''
                    state = States.IN_SVG
                elif '<path' in line:
                    line = line[line.find('<path') + 5:]
                    element = ''
                    state = States.IN_PATH
                elif '<circle' in line:
                    line = line[line.find('<circle') + 7:]
                    element = ''
                    state = States.IN_CIRCLE
                else:
                    line = ''
            elif state == States.IN_SVG:
                if '>' in line:
                    element += ' ' + line[:line.find('>')]
                    properties = shlex.split(element.strip())
                    for p in properties:
                        key_val_pair = p.split('=')
                        if len(key_val_pair) == 2:
                            svg_properties[key_val_pair[0]] = key_val_pair[1].strip()
                    line = line[line.find('>') + 1:]
                    state = States.SCANNING
                else:
                    element += ' ' + line
                    line = ''
            elif state == States.IN_PATH:
                if '/>' in line:
                    element += ' ' + line[:line.find('/>')]
                    path = {}
                    properties = shlex.split(element.strip())
                    for p in properties:
                        key_val_pair = p.split('=')
                        if len(key_val_pair) == 2:
                            path[key_val_pair[0]] = key_val_pair[1].strip()
                    paths.append(path)
                    line = line[line.find('/>') + 2:]
                    state = States.SCANNING
                else:
                    element += ' ' + line
                    line = ''
            elif state == States.IN_CIRCLE:
                if '/>' in line:
                    element += ' ' + line[:line.find('/>')]
                    circle = {}
                    properties = shlex.split(element.strip())
                    for p in properties:
                        key_val_pair = p.split('=')
                        if len(key_val_pair) == 2:
                            circle[key_val_pair[0]] = key_val_pair[1].strip()
                    line = line[line.find('/>') + 2:]
                    cx = float(circle['cx'])
                    cy = float(circle['cy'])
                    r = float(circle['r'])
                    d = 'M '
                    num_points = 24
                    dtheta = 2. * math.pi / num_points
                    for i in range(num_points):
                        d += f'{cx + r * math.cos(i * dtheta)},{cy + r * math.sin(i * dtheta)} '
                    d += 'Z'
                    path = {}
                    path['d'] = d
                    paths.append(path)
                    state = States.SCANNING
                else:
                    element += ' ' + line
                    line = ''

    svg_file.close()
    return [svg_properties, paths]

def parse_svg_paths(svg_paths):
    paths = []
    for p in svg_paths:
        try:
            d = p['d']
            d = d.replace(',', ' ')
            d = d.replace('M', 'M ')
            d = d.replace('m', 'm ')
            d = d.replace('L', 'L ')
            d = d.replace('l', 'l ')
            d = d.replace('H', 'H ')
            d = d.replace('h', 'h ')
            d = d.replace('V', 'V ')
            d = d.replace('v', 'v ')
            d = d.replace('A', 'A ')
            d = d.replace('a', 'a ')
            d = d.replace('  ', ' ')
            path_data = d.split(' ')
            path = []
            if path_data[0] != 'M' and path_data[0] != 'm':
                continue
            last_cmd = path_data[0]
            path.append(Point(float(path_data[1]), float(path_data[2])))
            i = 3
            while i < len(path_data):
                if path_data[i] == 'M':
                    last_cmd = path_data[i]
                    path.append(Point(float(path_data[i + 1]), float(path_data[i + 2])))
                    i += 3
                elif path_data[i] == 'm':
                    last_cmd = path_data[i]
                    path.append(path[-1] + Point(float(path_data[i + 1]), float(path_data[i + 2])))
                    i += 3
                elif path_data[i] == 'L':
                    last_cmd = path_data[i]
                    path.append(Point(float(path_data[i + 1]), float(path_data[i + 2])))
                    i += 3
                elif path_data[i] == 'l':
                    last_cmd = path_data[i]
                    path.append(path[-1] + Point(float(path_data[i + 1]), float(path_data[i + 2])))
                    i += 3
                elif path_data[i] == 'H':
                    last_cmd = path_data[i]
                    path.append(Point(float(path_data[i + 1]), path[-1].y))
                    i += 2
                elif path_data[i] == 'h':
                    last_cmd = path_data[i]
                    path.append(path[-1] + Point(float(path_data[i + 1]), 0.))
                    i += 2
                elif path_data[i] == 'V':
                    last_cmd = path_data[i]
                    path.append(Point(path[-1].x, float(path_data[i + 1])))
                    i += 2
                elif path_data[i] == 'v':
                    last_cmd = path_data[i]
                    path.append(path[-1] + Point(0., float(path_data[i + 1])))
                    i += 2
                elif path_data[i] == 'A' or path_data[i] == 'a':
                    last_cmd = path_data[i]
                    rx = float(path_data[i + 1])
                    ry = float(path_data[i + 2])
                    phi = math.pi * float(path_data[i + 3]) / 180.
                    largeArcFlag = bool(float(path_data[i + 4]))
                    sweepFlag = bool(float(path_data[i + 5]))
                    x1 = path[-1].x
                    y1 = path[-1].y
                    x2 = float(path_data[i + 6])
                    y2 = float(path_data[i + 7])
                    if last_cmd == 'a':
                        x2 += x1
                        y2 += y1

                    # See https://www.w3.org/TR/SVG/implnote.html#ArcImplementationNotes
                    cosPhi = math.cos(phi)
                    sinPhi = math.sin(phi)
                    xdm = 0.5 * (x1 - x2)
                    ydm = 0.5 * (y1 - y2)
                    x1_ = cosPhi * xdm + sinPhi * ydm
                    y1_ = -sinPhi * xdm + cosPhi * ydm
                    sqrtThingy = math.sqrt((rx ** 2 * ry ** 2 - rx ** 2 * y1_ ** 2 - ry ** 2 * x1_ ** 2) / (rx ** 2 * y1_ ** 2 + ry ** 2 * x1_ ** 2))
                    sqrtThingy = -sqrtThingy if largeArcFlag == sweepFlag else sqrtThingy
                    cx_ = sqrtThingy * rx * y1_ / ry
                    cy_ = -sqrtThingy * ry * x1_ / rx
                    cx = cosPhi * cx_ - sinPhi * cy_ + 0.5 * (x1 + x2)
                    cy = sinPhi * cx_ + cosPhi * cy_ + 0.5 * (y1 + y2)
                    theta = math.acos((x1_ - cx_) / (rx * math.sqrt(((x1_ - cx_) / rx) ** 2 + ((y1_ - cy_) / ry) ** 2)))
                    theta = -theta if y1_ - cy_ < 0. else theta
                    dtheta = math.acos(((x1_ - cx_) * (-x1_ - cx_) / rx ** 2 + (y1_ - cy_) * (-y1_ - cy_) / ry ** 2) / (math.sqrt(((x1_ - cx_) / rx) ** 2 + ((y1_ - cy_) / ry) ** 2) * math.sqrt(((-x1_ - cx_) / rx) ** 2 + ((-y1_ - cy_) / ry) ** 2)))
                    dtheta = -dtheta if (x1_ - cx_) * (-y1_ - cy_) - (y1_ - cy_) * (-x1_ - cx_) < 0. else dtheta
                    if not sweepFlag and dtheta > 0.:
                        dtheta -= 2. * math.pi
                    elif sweepFlag and dtheta < 0.:
                        dtheta += 2. * math.pi

                    numPoints = max(5, int(math.ceil(20. * abs(dtheta) / math.pi)))
                    dtheta = dtheta / (numPoints - 1)
                    for i in range(1, numPoints - 1):
                        path.append(Point(cx + rx * math.cos(theta + i * dtheta), cy + ry * math.sin(theta + i * dtheta)))
                    path.append(Point(x2, y2))
                    i += 8
                elif path_data[i] == 'Z' or path_data[i] == 'z':
                    break
                elif last_cmd == 'M':
                    path.append(Point(float(path_data[i]), float(path_data[i + 1])))
                    i += 2
                elif last_cmd == 'm':
                    path.append(path[-1] + Point(float(path_data[i]), float(path_data[i + 1])))
                    i += 2
                elif last_cmd == 'L':
                    path.append(Point(float(path_data[i]), float(path_data[i + 1])))
                    i += 2
                elif last_cmd == 'l':
                    path.append(path[-1] + Point(float(path_data[i]), float(path_data[i + 1])))
                    i += 2
                elif last_cmd == 'H':
                    path.append(Point(float(path_data[i]), path[-1].y))
                    i += 1
                elif last_cmd == 'h':
                    path.append(path[-1] + Point(float(path_data[i]), 0.))
                    i += 1
                elif last_cmd == 'V':
                    path.append(Point(path[-1].x, float(path_data[i])))
                    i += 1
                elif last_cmd == 'v':
                    path.append(path[-1] + Point(0., float(path_data[i])))
                    i += 1
                else:
                    break
            paths.append(path)
        except KeyError:
            pass
    return paths

def consolidate_paths(paths, radius):
    done = False
    while not done:
        initial_num_paths = len(paths)
        i = 0
        while i < len(paths):
            j = i + 1
            while j < len(paths):
                if (paths[i][-1] - paths[j][0]).mag() < radius:
                    paths[i].extend(paths[j][1:])
                    del paths[j]
                elif (paths[i][-1] - paths[j][-1]).mag() < radius:
                    paths[i].extend(paths[j][::-1][1:])
                    del paths[j]
                elif (paths[i][0] - paths[j][0]).mag() < radius:
                    paths[i] = paths[i][::-1]
                    paths[i].extend(paths[j][1:])
                    del paths[j]
                elif (paths[i][0] - paths[j][-1]).mag() < radius:
                    paths[i] = paths[i][::-1]
                    paths[i].extend(paths[j][::-1][1:])
                    del paths[j]
                else:
                    j += 1
            if (paths[i][0] - paths[i][-1]).mag() < radius:
                paths[i] = paths[i][:-1]
            i += 1
        done = len(paths) == initial_num_paths
    return paths

def consolidate_clusters(path, radius):
    new_path = []
    i = 0
    while i < len(path):
        done = False
        cluster = [path[i]]
        while not done:
            centroid = Point(sum([p.x for p in cluster]) / len(cluster), sum([p.y for p in cluster]) / len(cluster))
            if all([(p - centroid).mag() < radius for p in cluster]):
                if i + len(cluster) < len(path):
                    cluster.append(path[i + len(cluster)])
                else:
                    done = True
            else:
                cluster = cluster[:-1]
                done = True
        centroid = Point(sum([p.x for p in cluster]) / len(cluster), sum([p.y for p in cluster]) / len(cluster))
        new_path.append(centroid)
        i += len(cluster)
    return new_path

def is_convex(path):
    cross_products = []
    i = 0
    while i < len(path):
        p0 = path[(i - 1) % len(path)]
        p1 = path[i]
        p2 = path[(i + 1) % len(path)]

        x1 = p1.x - p0.x
        y1 = p1.y - p0.y

        x2 = p2.x - p1.x
        y2 = p2.y - p1.y

        cross_products.append(x1 * y2 - x2 * y1)
        i += 1
    return all([x < 0. for x in cross_products]) or all([x > 0. for x in cross_products])

def remove_self_intersections(path):
    remove_from_path = [False for p in path]
    new_path = [Point(p.x, p.y) for p in path]
    i = 0
    while i < len(new_path):
        a = new_path[i]
        b = new_path[(i + 1) % len(new_path)]
        j = 2
        while j < len(new_path) - 1:
            c = new_path[(i + j) % len(new_path)]
            d = new_path[(i + j + 1) % len(new_path)]
            r_num = (a.y - c.y) * (d.x - c.x) - (a.x - c.x) * (d.y - c.y)
            s_num = (a.y - c.y) * (b.x - a.x) - (a.x - c.x) * (b.y - a.y)
            denom = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x)
            if denom != 0:
                r = r_num / denom
                s = s_num / denom
                if (0. <= r <= 1.) and (0. <= s <= 1):
                    intersection = Point(a.x + r * (b.x - a.x), a.y + r * (b.y - a.y))
                    new_path[(i + 1) % len(new_path)] = intersection
                    k = 2
                    while k <= j:
                        remove_from_path[(i + k) % len(new_path)] = True
                        k += 1
                    i += j - 1
                    break
            j += 1
        i += 1
    i = len(new_path) - 1
    while i >= 0:
        if remove_from_path[i]:
            del(new_path[i])
        i -= 1
    return new_path

def offset_path(path, offset, radius):
    num_pos = 0
    num_neg = 0
    i = 0
    while i < len(path):
        p0 = path[(i - 1) % len(path)]
        p1 = path[i]
        p2 = path[(i + 1) % len(path)]

        x1 = p1.x - p0.x
        y1 = p1.y - p0.y

        x2 = p2.x - p1.x
        y2 = p2.y - p1.y

        magA = math.sqrt(x1 * x1 + y1 * y1)
        magB = math.sqrt(x2 * x2 + y2 * y2)
        AdotB = x1 * x2 + y1 * y2
        AcrossB = x1 * y2 - x2 * y1

        cosTheta = AdotB / (magA * magB)
        sinTheta = AcrossB / (magA * magB)
        theta = math.acos(cosTheta) if sinTheta >= 0. else -math.acos(cosTheta)
        if theta > 0.:
            num_pos += 1
        elif theta < 0.:
            num_neg += 1

        i += 1

    majority_sign = 1. if num_pos >= num_neg else -1.
    new_path = []
    i = 0
    while i < len(path):
        p0 = path[(i - 1) % len(path)]
        p1 = path[i]
        p2 = path[(i + 1) % len(path)]

        x1 = p1.x - p0.x
        y1 = p1.y - p0.y

        x2 = p2.x - p1.x
        y2 = p2.y - p1.y

        magA = math.sqrt(x1 * x1 + y1 * y1)
        magB = math.sqrt(x2 * x2 + y2 * y2)
        AdotB = x1 * x2 + y1 * y2
        AcrossB = x1 * y2 - x2 * y1

        cosTheta = AdotB / (magA * magB)
        sinTheta = AcrossB / (magA * magB)
        theta = math.acos(cosTheta) if sinTheta >= 0. else -math.acos(cosTheta)
        gamma = 0.5 * (math.pi - math.acos(cosTheta))
        offset_distance = offset / math.sin(gamma)
        if sinTheta < 0.:
            gamma = -gamma

        cosAlpha = x1 / magA
        sinAlpha = y1 / magA
        alpha = math.acos(cosAlpha) if sinAlpha >= 0. else -math.acos(cosAlpha)

        sign = math.copysign(1., theta) / majority_sign
        new_path.append(p1 + Point(sign * offset_distance * math.cos(alpha + theta + gamma), sign * offset_distance * math.sin(alpha + theta + gamma)))
        i += 1

    new_path = consolidate_clusters(new_path, radius)

    if is_convex(path) and not is_convex(new_path):
        return remove_self_intersections(new_path)
    else:
        return new_path

def offset_paths(paths, offset, radius):
    return [offset_path(path, offset, radius) for path in paths]

def bounding_box(paths):
    xvals = []
    yvals = []
    for path in paths:
        xvals.extend([p.x for p in path])
        yvals.extend([p.y for p in path])
    return (min(xvals), min(yvals), max(xvals), max(yvals))

def write_svg_file(svg_filename, width, height, scale_factor, origin, paths, path_colors, stroke_width):
    colors = {'r': '#FF0000', 'b': '#0000FF', 'k': '#000000'}
    svg_file = open(svg_filename, 'w')
    svg_file.write(f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="{width}cm" height="{height}cm" viewBox="0 0 {scale_factor * width} {scale_factor * height}">\n')
    for path, path_color in zip(paths, path_colors):
        svg_file.write(f'<path fill="none" stroke="{colors[path_color]}" stroke-width="{scale_factor * stroke_width}"\n')
        svg_file.write(f'  d="M {path[0].x - origin.x},{path[0].y - origin.y} L\n')
        i = 1
        while i < len(path):
            svg_file.write(f'  {path[i].x - origin.x},{path[i].y - origin.y}\n')
            i += 1
        svg_file.write('  Z" />\n')
    svg_file.write('</svg>')
    svg_file.close()

def convert_length(length):
    if length[-1] == '"': 
        return 2.54 * float(length[:-1])
    elif length[-2:] == 'in':
        return 2.54 * float(length[:-2])
    elif length[-3:] == 'mil':
        return 2.54 * float(length[:-3]) / 1000.
    elif length[-2:] == 'px':
        return 2.54 * float(length[:-2]) / 96.
    elif length[-2:] == 'pt':
        return 2.54 * float(length[:-2]) / 72.
    elif length[-2:] == 'cm':
        return float(length[:-2])
    elif length[-2:] == 'mm':
        return float(length[:-2]) / 10.
    elif length[-2:] == 'um' or length[-2:] == 'µm':
        return float(length[:-2]) / 10000.
    else:
        return float(length)

def make_stencil(project_name, **kwargs):
    edge_cuts_filename = kwargs.get('edge_cuts', project_name + '-Edge_Cuts.svg')
    f_paste_filename = kwargs.get('f_paste', project_name + '-F_Paste.svg')
    frame_filename = kwargs.get('frame', project_name + '-frame.svg')
    stencil_filename = kwargs.get('stencil', project_name + '-stencil.svg')
    show_cuts = True if kwargs.get('show_cuts', 'True') in ('True', 'TRUE', 'true', 'on', '1') else False
    show_paste = True if kwargs.get('show_paste', 'False') in ('True', 'TRUE', 'true', 'on', '1') else False
    stroke_width = convert_length(kwargs.get('stroke_width', '0.25px'))
    frame_offset = convert_length(kwargs.get('frame_offset', '1mm'))
    frame_width = convert_length(kwargs.get('frame_width', '6in'))
    frame_height = convert_length(kwargs.get('frame_height', '4in'))
    stencil_offset = convert_length(kwargs.get('stencil_offset', '75µm'))
    stencil_width = convert_length(kwargs.get('stencil_width', '4in'))
    stencil_height = convert_length(kwargs.get('stencil_height', '4in'))

    [edge_cuts_properties, edge_cuts_paths] = parse_svg_file(edge_cuts_filename)
    view_box = [float(val) for val in edge_cuts_properties['viewBox'].split(' ')]
    scale_factor = view_box[2] / convert_length(edge_cuts_properties['width'])
    edge_cuts = parse_svg_paths(edge_cuts_paths)
    edge_cut = consolidate_paths(edge_cuts, 5e-4 * scale_factor)
    edge_cut_bbox = bounding_box(edge_cut)
    edge_cut_center = Point(0.5 * (edge_cut_bbox[0] + edge_cut_bbox[2]), 0.5 * (edge_cut_bbox[1] + edge_cut_bbox[3]))

    [f_paste_properties, f_paste_paths] = parse_svg_file(f_paste_filename)
    f_paste = parse_svg_paths(f_paste_paths)
    f_paste = [path for path in f_paste if len(path) != 2]

    offset_edge_cut = offset_paths(edge_cut, -frame_offset * scale_factor, 5e-4 * scale_factor)

    frame_origin = Point(edge_cut_center.x - 0.5 * frame_width * scale_factor, edge_cut_center.y - 0.5 * frame_height * scale_factor)

    if show_cuts:
        write_svg_file(frame_filename, frame_width, frame_height, scale_factor, frame_origin, offset_edge_cut + edge_cut, list('b' + 'k'), stroke_width)
    else:
        write_svg_file(frame_filename, frame_width, frame_height, scale_factor, frame_origin, offset_edge_cut, list('b'), stroke_width)

    offset_f_paste = offset_paths(f_paste, stencil_offset * scale_factor, 5e-4 * scale_factor)

    stencil_origin = Point(edge_cut_center.x - 0.5 * stencil_width * scale_factor, edge_cut_center.y - 0.5 * stencil_height * scale_factor)
    if show_paste and show_cuts:
        write_svg_file(stencil_filename, stencil_width, stencil_height, scale_factor, stencil_origin, offset_f_paste + f_paste + edge_cut, list('b' * len(offset_f_paste) + 'k' * len(f_paste) + 'k' * len(edge_cut)), stroke_width)
    elif show_cuts:
        write_svg_file(stencil_filename, stencil_width, stencil_height, scale_factor, stencil_origin, offset_f_paste + edge_cut, list('b' * len(offset_f_paste) + 'k' * len(edge_cut)), stroke_width)
    elif show_paste:
        write_svg_file(stencil_filename, stencil_width, stencil_height, scale_factor, stencil_origin, offset_f_paste + f_paste, list('b' * len(offset_f_paste) + 'k' * len(f_paste)), stroke_width)
    else:
        write_svg_file(stencil_filename, stencil_width, stencil_height, scale_factor, stencil_origin, offset_f_paste, list('b' * len(offset_f_paste)), stroke_width)

if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] == '-h' or sys.argv[1] == '--help':
        print(f'''Usage: {sys.argv[0]} <project_name> [--<param1>=<val1> --<param2>=<val2>]

  Numerical values can be specified with units of in, mil, px, pt, cm, mm, or 
  µm (or um).  Numerical values supplied with no units are assumed to be in cm. 
  Valid parameter names and their default values are as follows:

    edge_cuts [<project_name>-Edge_Cuts.svg]: Name of the input file that 
            contains the edge cuts layer from the PCB.
    f_paste [<project_name>-F_Paste.svg]: Name of the input file that contains 
            the front paste layer from the PCB.
    frame [<project_name>-frame.svg]: Name of the ouput file that will contain 
            the cuts for the stencil frame.
    stencil [<project_name>-stencil.svg]: Name of the output file that will 
            contain the cuts for the stencil.
    show_cuts [True]: Switch to show original edge cuts layer in frame and 
            stencil.
    show_paste [False]: Switch to show original front paste layer in stencil.
    stroke_width [0.25px]: Stroke width to be used for the paths in the stencil 
            frame and stencil SVG files generated.
    frame_offset [1mm]: Distance to offset the edge cuts layer outward for the 
            cut-out in the stencil frame.
    frame_width [6in]: Width of the stencil frame.
    frame_height [4in]: Height of the stencil frame.
    stencil_offset [75µm]: Distance to offset the paths inward on the front 
            paste layer to account for the kerf of the laser in the stencil.
    stencil_width [4in]: Width of the stencil.
    stencil_height [4in]: Height of the stencil.''')
        sys.exit(1)
    kwargs = {}
    i = 2
    while i < len(sys.argv):
        if sys.argv[i][:2] == '--':
            arg = sys.argv[i][2:].split('=')
            if len(arg) == 2:
                kwargs[arg[0]] = arg[1]
        i += 1
    make_stencil(sys.argv[1], **kwargs)
    sys.exit(0)
