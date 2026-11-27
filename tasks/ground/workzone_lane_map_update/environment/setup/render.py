"""Synthetic dashcam observations from a calibrated pinhole camera."""

from PIL import Image, ImageDraw


SIZE = (640, 360)
NEAR = 0.65


def polygon(draw, camera, points, color):
    if hasattr(draw, "world_polygon"):
        draw.world_polygon(points, color)
        return
    points = [camera.camera_point(point) for point in points]
    clipped = []
    for a, b in zip(points, points[1:] + points[:1]):
        if a[2] >= NEAR:
            clipped.append(a)
        if (a[2] >= NEAR) != (b[2] >= NEAR):
            alpha = (NEAR - a[2]) / (b[2] - a[2])
            clipped.append(tuple(x + (y - x) * alpha for x, y in zip(a, b)))
    if len(clipped) >= 3:
        draw.polygon([camera.pixel(point) for point in clipped], fill=color)


def ribbon(draw, camera, path, width, color):
    half = width / 2
    left = [(x - half, y, 0) for x, y in path]
    right = [(x + half, y, 0) for x, y in reversed(path)]
    polygon(draw, camera, left + right, color)


def paint_arrow(draw, camera, x, y, toward):
    points = [(-0.10, -1.35), (0.10, -1.35), (0.10, 0.20),
              (0.48, 0.20), (0, 1.10), (-0.48, 0.20), (-0.10, 0.20)]
    polygon(draw, camera, [(x + dx + dy * toward, y + dy, 0) for dx, dy in points], "#f9eec6")


def cone(draw, camera, x, y):
    polygon(draw, camera, [(x - .31, y - .27, .01), (x + .31, y - .27, .01),
                           (x + .31, y + .27, .01), (x - .31, y + .27, .01)], "#393632")
    polygon(draw, camera, [(x - .23, y, 0), (x + .23, y, 0), (x + .04, y, .70),
                           (x - .04, y, .70)], "#ff7026")
    polygon(draw, camera, [(x - .15, y, .28), (x + .15, y, .28), (x + .12, y, .42),
                           (x - .12, y, .42)], "#fff4e0")


def lane_signals(draw, camera, signals, road):
    y = signals[0][1]
    for x in (-road, road):
        polygon(draw, camera, [(x - .10, y, 0), (x + .10, y, 0),
                               (x + .10, y, 4.6), (x - .10, y, 4.6)], "#68717a")
    polygon(draw, camera, [(-road, y, 4.42), (road, y, 4.42),
                           (road, y, 4.6), (-road, y, 4.6)], "#68717a")
    green_arrow = [(-.11, .29), (.11, .29), (.11, -.02), (.30, -.02),
                   (0, -.32), (-.30, -.02), (-.11, -.02)]
    red_cross = [(-.30, .19), (-.19, .30), (0, .11), (.19, .30), (.30, .19),
                 (.11, 0), (.30, -.19), (.19, -.30), (0, -.11), (-.19, -.30),
                 (-.30, -.19), (-.11, 0)]
    for x, y, is_open in signals:
        polygon(draw, camera, [(x - .46, y, 3.50), (x + .46, y, 3.50),
                               (x + .46, y, 4.47), (x - .46, y, 4.47)], "#222730")
        symbol, color = (green_arrow, "#54f58a") if is_open else (red_cross, "#f13755")
        polygon(draw, camera, [(x + dx, y - .01, 3.985 + dz) for dx, dz in symbol], color)


def survey_mark(draw, camera, x, y):
    ribbon(draw, camera, [(x, y - .98), (x, y + .98)], 1.08, "#222730")
    ribbon(draw, camera, [(x, y - .76), (x, y + .76)], .82, "#30b8df")


def construction_barriers(draw, camera, boundaries):
    pieces = []
    for key, direction in (("left", -1), ("right", 1)):
        line = boundaries[key]
        pieces.extend((b[1], i, a, b, direction) for i, (a, b) in enumerate(zip(line, line[1:]))
                      if a is not None and b is not None)
    for _, index, (x0, y0), (x1, y1), direction in sorted(pieces, reverse=True):
        thick = .32 * direction
        polygon(draw, camera, [(x0, y0, 0), (x1, y1, 0), (x1, y1, .68), (x0, y0, .68)],
                "#9b9e96" if index % 5 else "#7e857f")
        polygon(draw, camera, [(x0, y0, .68), (x1, y1, .68),
                               (x1 + thick, y1, .68), (x0 + thick, y0, .68)], "#cdd1c5")
        if index % 4 in (0, 1):
            polygon(draw, camera, [(x0, y0, .50), (x1, y1, .50),
                                   (x1, y1, .60), (x0, y0, .60)], "#eaa85d")


def workzone_paint(draw, camera, layout, painted):
    for rails in painted:
        for edge in rails.values():
            ribbon(draw, camera, [point for point in edge if point[1] >= .08 * layout["length"]],
                   .13, "#f0ce69")
        left, right = rails["left"], rails["right"]
        i = int(.91 * len(left))
        x, y = (left[i][0] + right[i][0]) / 2, left[i][1]
        paint_arrow(draw, camera, x, y, 0)


def draw_fixtures(draw, camera, fixtures):
    faces = []
    for fixture in fixtures:
        base = fixture["poly"]
        if fixture["kind"]=="milled_surface":
            polygon(draw,camera,[(x,y,.015) for x,y in base],"#989082")
            continue
        island = fixture["kind"] in ("island","channel_island")
        height = .23 if island else .68
        top = [(x, y, height) for x, y in base]
        # Sort individual faces where the gate meets the curb.
        def face(points, color):
            depth = sum(camera.camera_point(p)[2] for p in points) / len(points)
            faces.append((depth, points, color))
        face(top, "#aaa58b" if island else "#cdd1c5")
        for (x,y),(xx,yy) in zip(base, base[1:]+base[:1]):
            face([(x,y,0),(xx,yy,0),(xx,yy,height),(x,y,height)], "#827e70" if island else "#929892")
            if not island:
                face([(x,y,.46),(xx,yy,.46),(xx,yy,.55),(x,y,.55)], "#eaa85d")
    for _, points, color in sorted(faces, key=lambda item: -item[0]):
        polygon(draw, camera, points, color)


def scene(camera, layout, paths, cones, signals=(), painted=(), barriers=None, fixtures=(), canvas=None):
    image = Image.new("RGB", SIZE, "#b9cdd9") if canvas is None else None
    draw = ImageDraw.Draw(image) if canvas is None else canvas
    length = layout["length"]
    road = layout["road_half_width"]
    ribbon(draw, camera, [(0, -75), (0, 250)], 400, "#758267")
    ribbon(draw, camera, [(0, -75), (0, length + 80)], 2 * (road + .6), "#989082")
    ribbon(draw, camera, [(0, -75), (0, length + 80)], 2 * road, "#555961")
    for x in (-road + .25, road - .25):
        ribbon(draw, camera, [(x, -75), (x, length + 80)], .12, "#dad9cc")
    half_lane = layout["lane_width"] / 2
    if "access_edges" in layout:
        for old in layout.get("previous_edges",()):
            ribbon(draw,camera,old,.18,"#a59b7e")
        for edge in layout["access_edges"].values():
            extended = [(edge[0][0], -36), *edge, (edge[-1][0], length+60)]
            ribbon(draw, camera, extended, .14, "#f0ce69")
        for x,y,slope in layout["access_arrows"]:
            paint_arrow(draw, camera, x, y, slope)
    else:
        for x in (-half_lane, half_lane):
            for y in range(-36, int(length) + 60, 6):
                ribbon(draw, camera, [(x, y), (x, y + 3)], .11, "#d0d2ca")

    for path in paths:
        shift_start = layout["length"] * .10
        guide = [(x, y) for x, y in path if y > shift_start]
        # Start the temporary paint after the initial approach and keep the old stripes visible.
        for side in (-1, 1):
            edge = [(x + side * layout["lane_width"] / 2, y) for x, y in guide]
            ribbon(draw, camera, edge, .13, "#f0ce69")
        a, b = path[-2], path[-1]
        paint_arrow(draw, camera, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2,
                    (b[0] - a[0]) / (b[1] - a[1]))

    if painted:
        workzone_paint(draw, camera, layout, painted)
    for x, y in sorted(cones, key=lambda point: -point[1]):
        cone(draw, camera, x, y)
    if barriers:
        construction_barriers(draw, camera, barriers)
    for x, y in layout.get("survey_marks", ()):
        survey_mark(draw, camera, x, y)
    if fixtures:
        draw_fixtures(draw, camera, fixtures)
    if signals:
        lane_signals(draw, camera, signals, road)
    return image
