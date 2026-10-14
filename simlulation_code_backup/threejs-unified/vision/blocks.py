"""Finds coloured cubes on the mat and on top of each other, from one RGB frame.

No depth: the OAK stereo depth is not usable. Everything comes from the printed
ArUco mat (lutich/robot-arm, config/camera-table-xy/calibration.json) and the known
cube size. Method in docs/tracking-plan.md:

1. Detect the mat markers. Fit the camera (focal length and pose) to them every
   frame, so a bumped camera does not break anything.
2. Segment each cube colour in HSV.
3. Stack level: a cube whose front-bottom edge borders another cube in the image
   rests on that cube; otherwise it rests on the mat.
4. Map the cube outline onto the plane of its bottom face (Z = level x 26,5 mm).
   Points of the cube above that plane land further from the camera, so the outline
   points with the largest X are the front-bottom edge. Centre = middle of that edge,
   moved half a cube away from the camera.

Coordinates are the camera table frame: mm, origin at the mat edge nearest the arm,
+X away from the arm, +Y to the left seen from the arm, Z up.

  .venv/Scripts/python vision/blocks.py docs/images/photos/controller-camera-2026-10-12.jpg --out annotated.png
  .venv/Scripts/python vision/blocks.py --url http://10.102.4.103:8765 --every 5     (GET only)
"""
import argparse, json, math, os, sys, time, urllib.request
import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CAL = os.path.join(os.path.dirname(REPO), "robot-arm", "config", "camera-table-xy", "calibration.json")
CUBE_MM = 26.5
W, H_IMG = 1280, 720

# OpenCV HSV ranges (H 0-179). Only "orange" is tuned, on the 2026-10-12 frames.
# The others are untested starting points; check them on a real frame before use.
COLOURS = {
    "orange": [((3, 120, 110), (20, 255, 255))],
    "red":    [((0, 140, 90), (2, 255, 255)), ((170, 140, 90), (179, 255, 255))],
    "yellow": [((21, 120, 120), (35, 255, 255))],
    "green":  [((40, 80, 60), (85, 255, 255))],
    "blue":   [((95, 120, 60), (130, 255, 255))],
}
MIN_AREA_PX = 300
EDGE_TOL_MM = 4.0      # outline points this close to the largest X belong to the front-bottom edge
EDGE_OK_MM = 4.0       # front edge must measure CUBE_MM within this, otherwise the cube is occluded


# ---------------- camera model from the mat markers ----------------

def load_layout(path):
    cal = json.load(open(path))
    return {m["id"]: np.array(m["corners_xy_mm"], float) for m in cal["table_layout"]["markers"]}


def detect_markers(img, layout):
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    return {int(i): c[0].astype(float) for c, i in zip(corners, [] if ids is None else ids.ravel()) if i in layout}


def fit_camera(found, layout):
    """Pinhole camera from the visible mat markers: focal length, principal point, pose.

    The image is already undistorted by the camera, so no distortion terms are fitted.
    Needs at least 2 markers (8 corners); 4 or more spread over the mat are better.
    """
    if len(found) < 2:
        raise ValueError(f"need at least 2 mat markers, saw {sorted(found)}")
    ids = sorted(found)
    obj = np.vstack([np.c_[layout[i], np.zeros(4)] for i in ids]).astype(np.float32)
    img = np.vstack([found[i] for i in ids]).astype(np.float32)
    K0 = np.array([[950.0, 0, W / 2], [0, 950.0, H_IMG / 2], [0, 0, 1]])
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST
             | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
    if len(found) < 4:
        flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    rms, K, _, rv, tv = cv2.calibrateCamera([obj], [img], (W, H_IMG), K0, np.zeros(5), flags=flags)
    R, _ = cv2.Rodrigues(rv[0])
    return dict(K=K, R=R, t=tv[0].ravel(), rms_px=float(rms), markers=ids,
                centre_mm=(-R.T @ tv[0]).ravel())


def image_to_plane(cam, pts, z_mm):
    """Image points to table X, Y on the horizontal plane Z = z_mm."""
    K, R, t = cam["K"], cam["R"], cam["t"]
    G = K @ np.c_[R[:, 0], R[:, 1], z_mm * R[:, 2] + t]          # plane (X, Y, 1) -> image
    p = cv2.perspectiveTransform(np.asarray(pts, np.float64).reshape(-1, 1, 2), np.linalg.inv(G))
    return p.reshape(-1, 2)


def project(cam, xyz):
    p = cam["K"] @ (cam["R"] @ np.asarray(xyz, float).T + cam["t"][:, None])
    return (p[:2] / p[2]).T


# ---------------- cubes ----------------

def colour_blobs(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blobs, masks = [], {}
    for name, ranges in COLOURS.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        masks[name] = mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for c in contours:
            if cv2.contourArea(c) >= MIN_AREA_PX:
                blobs.append(dict(colour=name, hull=cv2.convexHull(c).reshape(-1, 2).astype(float),
                                  area_px=float(cv2.contourArea(c))))
    return blobs, masks


def front_edge(cam, hull, z_mm):
    """Front-bottom edge of a cube whose bottom face is at z_mm: two table points and the centre."""
    xy = image_to_plane(cam, hull, z_mm)
    front = xy[xy[:, 0] >= xy[:, 0].max() - EDGE_TOL_MM]
    a, b = front[np.argmin(front[:, 1])], front[np.argmax(front[:, 1])]
    edge = b - a
    width = float(np.linalg.norm(edge))
    n = np.array([-edge[1], edge[0]]) / max(width, 1e-9)
    if n[0] > 0:
        n = -n                                                     # away from the camera
    return dict(a=a, b=b, width_mm=width, centre=(a + b) / 2 + n * CUBE_MM / 2,
                yaw_deg=math.degrees(math.atan2(-edge[0], edge[1])))


def resting_on(blob, blobs, masks, img_shape):
    """Index of the cube this one rests on, or None for the mat.

    Looks at a thin strip just below the blob's bottom outline in the image (toward
    the camera). If most of it has another cube's colour, this cube sits on that cube.
    """
    h = blob["hull"]
    bottom = h[h[:, 1] >= h[:, 1].max() - 6]
    x0, x1 = int(bottom[:, 0].min()), int(bottom[:, 0].max())
    y = int(h[:, 1].max())
    y0, y1 = min(y + 3, img_shape[0] - 1), min(y + 10, img_shape[0])
    if x1 <= x0 or y1 <= y0:
        return None
    best, best_frac = None, 0.5
    for j, other in enumerate(blobs):
        if other is blob:
            continue
        strip = masks[other["colour"]][y0:y1, x0:x1]
        inside = np.zeros_like(strip)
        poly = (other["hull"] - [x0, y0]).astype(np.int32)
        cv2.fillPoly(inside, [poly], 255)
        frac = float(np.count_nonzero(strip & inside)) / strip.size
        if frac > best_frac:
            best, best_frac = j, frac
    return best


def cube_outline(cam, x, y, z0, yaw_deg):
    """Image outline of a CUBE_MM cube with its bottom centre at (x, y, z0)."""
    s, r = CUBE_MM / 2, math.radians(yaw_deg)
    R2 = np.array([[math.cos(r), -math.sin(r)], [math.sin(r), math.cos(r)]])
    sq = (R2 @ np.array([[s, s], [s, -s], [-s, -s], [-s, s]]).T).T + [x, y]
    pts = np.vstack([np.c_[sq, np.full(4, z)] for z in (z0, z0 + CUBE_MM)])
    return cv2.convexHull(project(cam, pts).astype(np.float32)).reshape(-1, 2)


def size_level(cam, hull, max_level=3):
    """Stack level from the known cube size alone.

    For each level, place a cube on the front-bottom edge at that height, project the
    whole cube and compare its outline with the blob (intersection over union).
    A cube one level too low or too high comes out about 7 % too large or small.
    """
    blob = np.zeros((H_IMG, W), np.uint8)
    cv2.fillPoly(blob, [hull.astype(np.int32)], 1)
    scores = []
    for k in range(max_level + 1):
        e = front_edge(cam, hull, k * CUBE_MM)
        model = np.zeros_like(blob)
        cv2.fillPoly(model, [cube_outline(cam, *e["centre"], k * CUBE_MM, e["yaw_deg"]).astype(np.int32)], 1)
        inter, union = np.count_nonzero(blob & model), np.count_nonzero(blob | model)
        scores.append(inter / union if union else 0.0)
    return int(np.argmax(scores)), scores


def find_cubes(img, cam):
    blobs, masks = colour_blobs(img)
    below = [resting_on(b, blobs, masks, img.shape) for b in blobs]

    def level(i, seen=()):
        if below[i] is None or i in seen:
            return 0
        return level(below[i], seen + (i,)) + 1

    cubes = []
    for i, b in enumerate(blobs):
        k = level(i)
        # The size check needs the whole cube outline; a cube on top hides this one's top face.
        covered = i in below
        k_size, iou = (None, []) if covered else size_level(cam, b["hull"])
        e = front_edge(cam, b["hull"], k * CUBE_MM)
        cubes.append(dict(colour=b["colour"], level=k, level_from_size=k_size,
                          iou_per_level=[round(v, 3) for v in iou],
                          on=None if below[i] is None else blobs[below[i]]["colour"],
                          x_mm=float(e["centre"][0]), y_mm=float(e["centre"][1]),
                          bottom_z_mm=k * CUBE_MM, top_z_mm=(k + 1) * CUBE_MM,
                          yaw_deg=float(e["yaw_deg"]), front_edge_mm=e["width_mm"],
                          valid=abs(e["width_mm"] - CUBE_MM) <= EDGE_OK_MM,
                          _hull=b["hull"], _edge=(e["a"], e["b"])))
    return cubes


# ---------------- output ----------------

def board_frame(x_mm, y_mm, d_mm):
    """Camera table frame (mm) to the board frame of docs/arm-dimensions.md (cm).

    d_mm: distance from the S1 axis to the mat's near edge along +X (not measured yet).
    """
    return -y_mm / 10, (x_mm + d_mm) / 10


def annotate(img, cam, cubes):
    out = img.copy()
    for c in cubes:
        colour = (0, 200, 0) if c["valid"] else (0, 0, 255)
        cv2.polylines(out, [c["_hull"].astype(np.int32)], True, colour, 2)
        a, b = c["_edge"]
        pa, pb = project(cam, [[*a, c["bottom_z_mm"]], [*b, c["bottom_z_mm"]]])
        cv2.line(out, tuple(map(int, pa)), tuple(map(int, pb)), (255, 255, 0), 3)
        # Draw the fitted cube as a wireframe, to check it lines up with the real one.
        x, y, z0 = c["x_mm"], c["y_mm"], c["bottom_z_mm"]
        s, r = CUBE_MM / 2, math.radians(c["yaw_deg"])
        R2 = np.array([[math.cos(r), -math.sin(r)], [math.sin(r), math.cos(r)]])
        sq = (R2 @ np.array([[s, s], [s, -s], [-s, -s], [-s, s]]).T).T + [x, y]
        for z in (z0, z0 + CUBE_MM):
            pts = project(cam, np.c_[sq, np.full(4, z)]).astype(np.int32)
            cv2.polylines(out, [pts], True, (255, 0, 255), 1)
        if c["level_from_size"] is not None and c["level"] != c["level_from_size"]:
            colour = (0, 0, 255)
        label = (f"{c['colour']} L{c['level']} X={x:.0f} Y={y:.0f}" + ("" if c["valid"] else " occluded?")
                 + ("" if c["level_from_size"] in (None, c["level"]) else f" size says L{c['level_from_size']}"))
        tx, ty = c["_hull"].min(0).astype(int)
        cv2.putText(out, label, (tx, max(15, ty - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
    cv2.putText(out, f"camera fit {cam['rms_px']:.2f} px rms, f={cam['K'][0, 0]:.0f}, markers {cam['markers']}",
                (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return out


def analyse(img, layout, d_mm=None):
    cam = fit_camera(detect_markers(img, layout), layout)
    cubes = find_cubes(img, cam)
    report = dict(camera=dict(rms_px=round(cam["rms_px"], 2), f_px=round(float(cam["K"][0, 0]), 1),
                              markers=cam["markers"], centre_mm=[round(float(v), 1) for v in cam["centre_mm"]]),
                  cubes=[])
    for c in cubes:
        r = {k: (round(v, 1) if isinstance(v, float) else v) for k, v in c.items() if not k.startswith("_")}
        if d_mm is not None:
            r["board_cm"] = [round(v, 2) for v in board_frame(c["x_mm"], c["y_mm"], d_mm)]
        report["cubes"].append(r)
    return cam, cubes, report


def fetch(url):
    with urllib.request.urlopen(url.rstrip("/") + "/api/camera/snapshot.jpg", timeout=20) as r:
        return cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", nargs="?", help="saved camera image (1280 x 720)")
    ap.add_argument("--url", help="controller base URL; fetches /api/camera/snapshot.jpg (GET only)")
    ap.add_argument("--every", type=float, default=0, help="with --url: repeat every N seconds")
    ap.add_argument("--calibration", default=DEFAULT_CAL, help="calibration.json with the mat layout")
    ap.add_argument("--out", help="write an annotated image here")
    ap.add_argument("--s1-offset-mm", type=float, help="S1 axis to the mat's near edge, to also print board cm")
    args = ap.parse_args()
    if not (args.image or args.url):
        ap.error("give an image or --url")
    layout = load_layout(args.calibration)

    while True:
        img = cv2.imread(args.image) if args.image else fetch(args.url)
        if img is None:
            sys.exit("could not read the image")
        cam, cubes, report = analyse(img, layout, args.s1_offset_mm)
        print(json.dumps(report))
        if args.out:
            cv2.imwrite(args.out, annotate(img, cam, cubes))
        if not (args.url and args.every > 0):
            break
        time.sleep(args.every)


if __name__ == "__main__":
    main()
