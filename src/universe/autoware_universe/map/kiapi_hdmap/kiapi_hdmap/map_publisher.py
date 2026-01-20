#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
🗺️  HD-Map → RViz2  (UTM52N 폴더 자동 탐색 · 오프셋 공유)
ROS 2 Humble · Python ≥ 3.10

- B2_SURFACELINEMARK: Type 코드 기반 색/패턴(dash) 렌더링
- B3_SURFACEMARK:
  - Type=5, Kind=5321 (횡단보도): 스트라이프 흰색 채움
  - Type=1 (화살표): Kind 기반으로 직진/좌/우/직+좌/직+우를 "꺾이는" 곡선 화살표로 생성하여 흰색 채움
"""

from __future__ import annotations
import re, unicodedata, yaml, math
import numpy as np
from pathlib import Path
from typing  import Dict, List, Optional, Tuple

import rclpy, tf2_ros, geopandas as gpd
from rclpy.node  import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg      import Point, TransformStamped, Quaternion
from std_msgs.msg           import ColorRGBA

from shapely.geometry import Polygon, MultiPolygon, LineString as SLineString, Point as SPoint
from shapely.ops      import triangulate, unary_union
from shapely.affinity import rotate as shp_rotate, translate as shp_translate, scale as shp_scale

# ================================================================
# SECTION GUIDE (수정 포인트 빠르게 찾기)
#  1) 사용자 설정/파라미터: DATA_ROOT, FRAME_ID, PUBLISH_HZ, B2/B3/TL_* 상수
#  2) 공통 유틸: 색/오프셋/좌표계 보정, yaw/quaternion 변환, shapely 변환 helpers
#  3) C1(신호등) 렌더링: head_only, mast-arm, traffic_light_markers_v1/v2
#  4) B2/B3 렌더링: dash/line/arrow/crosswalk 생성 로직
#  5) GeoDataFrame → MarkerArray 변환: _gdf2markers
#  6) ROS Node: HDMapPublisher (데이터 로드, TF, 퍼블리시)
# ================================================================

# ──────────────── 사용자 설정 ─────────────────────────────
# DATA_ROOT  = Path("/home/kiapi/Downloads/new좌표계") #원본
# DATA_ROOT  = Path("/home/aiict/KIAPI_ioniq5/map_data/KIAPI_SHP") # shp 파일 저장소
FRAME_ID   = "map"
PUBLISH_HZ = 2.0

# ===== 선 두께 파라미터(권장) =====
LINE_W_DEFAULT = 0.18
LINE_W_B2      = 0.30
LINE_W_POLY    = 0.20
POINT_R        = 0.60
# B3 렌더 파라미터
B3_OUTLINE_W   = 0.08
B3_FILL_ALPHA  = 1.0

# ===== B3 Arrow style tuning (중요) =====
ARROW_L_RATIO = 0.90
# 화살표 전체 길이 L을 footprint long_L 대비 비율
ARROW_L_MIN   = 3.8
# 길이 하한
ARROW_L_MAX   = 7.2
# 길이 상한

ARROW_BODY_W_RATIO = 0.22
# 몸통 두께를 short_L 대비 비율
ARROW_BODY_W_MIN   = 0.30
# 몸통 두께 하한
ARROW_BODY_W_MAX   = 1.05
# 몸통 두께 상한

ARROW_HEAD_L_RATIO = 0.65
# 화살촉 길이를 L 대비 비율(길게)
ARROW_HEAD_L_MIN   = 1.05
# 화살촉 길이 하한
ARROW_HEAD_L_MAX   = 3.2
# 화살촉 길이 상한

ARROW_HEAD_W_RATIO = 2.8
# 화살촉 폭을 몸통 대비 비율(넓게)
ARROW_HEAD_W_MIN   = 0.90
# 화살촉 폭 하한
ARROW_HEAD_W_MAX   = 3.4
# 화살촉 폭 상한

FORK_ATTACH_RATIO   = 0.55
# 분기 시작 위치(몸통 길이 비율)
FORK_PRE_RATIO      = 0.18
# 분기 시작 후 직진 구간 비율
FORK_ARM_RATIO      = 0.40
# 옆으로 빠지는 팔 길이 비율
FORK_BRANCH_W_RATIO = 0.85
# 분기 팔 두께(몸통 대비)

# 오프셋 직접 지정
X_OFFSET = 445863.1 # KIAPI 원형교차로 입구 원점
Y_OFFSET = 3944970.8 # KIAPI 원형교차로 입구 원점
# ─────────────────────────────────────────────────────────

OFF_YAML = Path(__file__).with_name("offset.yaml")

# ---------- 색상 팔레트 ---------------------------------
def _hex(h: str, a: float = 1.0) -> ColorRGBA:
    h = h.lstrip('#')
    r,g,b = [int(h[i:i+2], 16)/255 for i in (0,2,4)]
    return ColorRGBA(r=r, g=g, b=b, a=a)

COLOR10 = [_hex(c) for c in (
    "#f94144","#f3722c","#f8961e","#f9c74f","#90be6d",
    "#43aa8b","#4d908e","#577590","#277da1","#9c89b8")]

def slug(t: str) -> str:
    s = unicodedata.normalize("NFKD", t).encode("ascii","ignore").decode()
    s = re.sub(r"[^\w]+","_",s).strip("_").lower()
    s = re.sub(r"_+","_",s)
    return f"n{s}" if s and s[0].isdigit() else (s or "layer")

def _rgba(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))

# ===== A5 ParkingLot style + synthetic slots =====
A5_OUTLINE_W    = 0.18
A5_Z_LINE       = 0.028
A5_Z_FILL       = 0.020
A5_COL_WHITE    = _rgba(0.98, 0.98, 0.98, 1.0)

A5_DRAW_FILL    = False
A5_FILL_ALPHA   = 0.10
A5_FILL_COL     = _rgba(0.85, 0.85, 0.85, 1.0)

A5_SYNTH_SLOTS_ENABLE = True
A5_SLOT_N      = 6
A5_SLOT_GAP    = 0.25
A5_SLOT_MARGIN = 0.25
A5_SLOT_DEPTH_RATIO = 0.85
A5_SLOT_MIN_W  = 1.8
A5_SLOT_MIN_D  = 3.8
A5_SLOT_MAX_D  = 6.2

# B2 Type 색상 팔레트
B2_COL_YELLOW = _rgba(1.00, 0.86, 0.10, 1.0)
B2_COL_WHITE  = _rgba(0.95, 0.95, 0.95, 1.0)
B2_COL_BLUE   = _rgba(0.20, 0.60, 1.00, 1.0)
B2_COL_OTHER  = _rgba(0.70, 0.70, 0.70, 1.0)

# B3 횡단보도 화살표 색상 "흰색 고정"
B3_COL_WHITE  = _rgba(0.98, 0.98, 0.98, 1.0)

# ===== C1 Traffic Light style =====
TL_POLE_H      = 4.8
TL_POLE_R      = 0.08

TL_BOX_W_V     = 0.38
TL_BOX_D       = 0.26
TL_BOX_H_V_3   = 1.05
TL_BOX_H_V_4   = 1.35

TL_BOX_W_H_3   = 1.05
TL_BOX_W_H_4   = 1.35
TL_BOX_H_H     = 0.38

TL_LAMP_R      = 0.10
TL_LAMP_GAP    = 0.28
TL_LAMP_INSET  = 0.035   # 램프를 박스 면에서 안쪽으로 넣는 정도
TL_LAMP_EPS    = 0.006   # 앞/뒤 중복 시 z-fighting 방지용 미세 오프셋

TL_COL_POLE = _rgba(0.15, 0.15, 0.15, 1.0)
TL_COL_BOX  = _rgba(0.05, 0.05, 0.05, 1.0)

TL_COL_R    = _rgba(0.95, 0.10, 0.10, 1.0)
TL_COL_Y    = _rgba(0.95, 0.85, 0.10, 1.0)
TL_COL_G    = _rgba(0.10, 0.95, 0.20, 1.0)
TL_COL_W    = _rgba(0.95, 0.95, 0.95, 1.0)

# 보행자 신호 전용 낮은 높이 오프셋
TL_PED_LOWER = 1.20
# 보행자 신호 방향 보정: 왼쪽으로 90deg
TL_PED_YAW_OFFSET = math.pi / 2.0

# ===== C3 Vehicle Protection Safety (curb/rail/barrier 등) =====
C3_RESAMPLE_STEP = 0.25  # [m] 라인 리샘플 간격(더 촘촘)
C3_Z_BASE        = 0.06  # [m] 바닥 오프셋(z-fighting 방지, B2 z=0.02와 분리)

# 윤곽선(Outline) 옵션
C3_OUTLINE_ENABLE = True
C3_OUTLINE_W      = 0.14
C3_OUTLINE_Z_ADD  = 0.06
C3_OUTLINE_COL    = _rgba(0.05, 0.05, 0.05, 1.0)

# C3 포스트(rail/barrier만)
C3_POST_ENABLE = True
C3_POST_EVERY_M = 3.0
C3_POST_SIZE_W  = 0.18
C3_POST_SIZE_H  = 0.55
C3_POST_COL     = _rgba(0.10, 0.10, 0.10, 1.0)

# Type별 스타일 (width, height, color, alpha)
C3_STYLE = {
    4: dict(name="curb",    w=0.35, h=0.22, col=_rgba(0.50, 0.50, 0.50, 1.0), a=1.0),  # 연석: 회색
    2: dict(name="rail",    w=0.22, h=0.95, col=_rgba(0.00, 1.00, 1.00, 1.0), a=1.0),  # 가드레일
    3: dict(name="barrier", w=0.45, h=1.20, col=_rgba(1.00, 0.45, 0.00, 1.0), a=1.0),  # 방호벽
    7: dict(name="temp",    w=0.30, h=0.90, col=_rgba(1.00, 0.00, 1.00, 1.0), a=1.0),  # 임시구조물
    99:dict(name="facility",w=0.50, h=3.20, col=_rgba(0.65, 0.65, 0.70, 1.0), a=0.90), # 기타 시설물(건물 후보)
}
C3_DEFAULT = dict(name="unknown", w=0.24, h=0.30, col=_rgba(0.55, 0.55, 0.55, 1.0), a=0.9)

# ===== C1 Mast-arm (2-head) grouping params =====
C1_PAIR_DIST_MAX = 25.0
C1_PAIR_DIST_MIN = 0.3
C1_PAIR_YAW_MAX_DEG = 25.0
C1_PAIR_LATERAL_RATIO = 0.7

TL_ARM_THICK = 0.08
TL_ARM_DROP_Z = 0.10

# C1 ↔ C6/Pose 연관 파라미터
POST_ASSOC_MAX = 12.0  # C1 헤드 ↔ C6 포스트 허용 거리
A2_YAW_SEARCH_R = 8.0  # A2 tangent yaw 탐색 반경

# ===== C1 Traffic Light 방향 보정(우측만 flip) =====
C1_FLIP_RIGHT_SIDE = False   # center 기반 yaw 사용 시 비활성
C1_SPLIT_MODE = "median_x"   # "median_x" 또는 "mean_x"
C1_SPLIT_X_MANUAL = None     # float 값 지정 시 그 값을 기준선으로 사용
C1_FLIP_DEBUG_LOG = True

# ================================================================
# Attribute & 값 파싱 유틸
# ================================================================
def _colname_case_insensitive(gdf, key: str) -> str | None:
    k = key.lower()
    for c in gdf.columns:
        if c.lower() == k:
            return c
    return None

def _get_int(row, col: Optional[str]) -> Optional[int]:
    if col is None:
        return None
    v = row.get(col, None)
    if v is None:
        return None
    try:
        s = str(v).strip()
        if s == "" or s.lower() == "nan":
            return None
        return int(float(s))
    except Exception:
        return None

def _get_float(row, col: Optional[str]) -> Optional[float]:
    if col is None:
        return None
    v = row.get(col, None)
    if v is None:
        return None
    try:
        s = str(v).strip()
        if s == "" or s.lower() == "nan":
            return None
        return float(s)
    except Exception:
        return None


def _find_heading_col(gdf) -> str | None:
    for key in ("Heading", "HEADING", "Angle", "ANGLE", "Yaw", "YAW",
                "Azimuth", "AZIMUTH", "Dir", "DIR", "Direction", "DIRECTION", "Rot", "ROT"):
        c = _colname_case_insensitive(gdf, key)
        if c is not None:
            return c
    return None


def _wrap_pi(a: float) -> float:
    while a >= math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _angle_between(v1: tuple[float, float], v2: tuple[float, float]) -> float:
    x1, y1 = v1
    x2, y2 = v2
    n1 = math.hypot(x1, y1)
    n2 = math.hypot(x2, y2)
    if n1 < 1e-9 or n2 < 1e-9:
        return math.pi
    c = (x1*x2 + y1*y2) / (n1*n2)
    c = max(-1.0, min(1.0, c))
    return math.acos(c)


def _angdiff(a: float, b: float) -> float:
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d


def _angdist(a: float, b: float) -> float:
    return abs(_angdiff(a, b))


def _yaw_mean(a: float, b: float) -> float:
    """원형 평균"""
    return math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))


def _yaw_from_heading_value(hv: float, yaw_geom: float) -> float:
    raw = float(hv)
    raws = []
    if abs(raw) <= 2.0 * math.pi + 0.2:
        raws.append(raw)
    raws.append(math.radians(raw))

    cands = []
    for r in raws:
        cands.append(_wrap_pi(r))
        cands.append(_wrap_pi((math.pi / 2.0) - r))
        cands.append(_wrap_pi(r - (math.pi / 2.0)))

    yg0 = _wrap_pi(yaw_geom)
    yg1 = _wrap_pi(yaw_geom + math.pi)

    best = None
    for y in cands:
        cost = min(_angdist(y, yg0), _angdist(y, yg1))
        if best is None or cost < best[0]:
            best = (cost, y)
    return best[1] if best else yg0

def _yaw_from_heading_azimuth(hv: float, yaw_geom: float) -> float:
    return _yaw_from_heading_value(hv, yaw_geom)


def _direct_yaw_by_pointiness(poly: Polygon, yaw_guess: float) -> float:
    coords = list(poly.exterior.coords)
    if len(coords) < 6:
        return _wrap_pi(yaw_guess)
    coords = coords[:-1]
    n = len(coords)

    ux, uy = math.cos(yaw_guess), math.sin(yaw_guess)
    s = [ux*x + uy*y for (x, y) in coords]
    smin, smax = min(s), max(s)
    L = smax - smin
    if L < 1e-3:
        return _wrap_pi(yaw_guess)

    frac = 0.10
    hi = [i for i, val in enumerate(s) if val >= smax - frac*L]
    lo = [i for i, val in enumerate(s) if val <= smin + frac*L]

    def min_turn_angle(idxs: list[int]) -> float:
        if not idxs:
            return math.pi
        best = math.pi
        for i in idxs:
            p_prev = coords[(i - 1) % n]
            p      = coords[i]
            p_next = coords[(i + 1) % n]
            v1 = (p_prev[0] - p[0], p_prev[1] - p[1])
            v2 = (p_next[0] - p[0], p_next[1] - p[1])
            ang = _angle_between(v1, v2)
            if ang < best:
                best = ang
        return best

    a_hi = min_turn_angle(hi)
    a_lo = min_turn_angle(lo)

    yaw = yaw_guess
    if a_lo < a_hi:
        yaw += math.pi
    return _wrap_pi(yaw)


def _direct_yaw_if_asymmetric(poly: Polygon, yaw_guess: float) -> float:
    coords = list(poly.exterior.coords)
    if len(coords) < 6:
        return _wrap_pi(yaw_guess)

    coords = coords[:-1]
    n = len(coords)
    ux, uy = math.cos(yaw_guess), math.sin(yaw_guess)
    s = [ux*x + uy*y for (x, y) in coords]
    smin, smax = min(s), max(s)
    L = smax - smin
    if L < 1e-3:
        return _wrap_pi(yaw_guess)

    frac = 0.10
    hi = [i for i, val in enumerate(s) if val >= smax - frac*L]
    lo = [i for i, val in enumerate(s) if val <= smin + frac*L]

    def min_turn_angle(idxs: list[int]) -> float:
        if not idxs:
            return math.pi
        best = math.pi
        for i in idxs:
            p_prev = coords[(i - 1) % n]
            p      = coords[i]
            p_next = coords[(i + 1) % n]
            v1 = (p_prev[0] - p[0], p_prev[1] - p[1])
            v2 = (p_next[0] - p[0], p_next[1] - p[1])
            ang = _angle_between(v1, v2)
            best = min(best, ang)
        return best

    a_hi = min_turn_angle(hi)
    a_lo = min_turn_angle(lo)

    if abs(a_hi - a_lo) < 0.25:
        return _wrap_pi(yaw_guess)

    return _direct_yaw_by_pointiness(poly, yaw_guess)


def _robust_split_x(xs: list[float]) -> float:
    if not xs:
        return 0.0
    if C1_SPLIT_X_MANUAL is not None:
        return float(C1_SPLIT_X_MANUAL)
    if C1_SPLIT_MODE == "mean_x":
        return float(sum(xs) / len(xs))
    xs2 = sorted(xs)
    n = len(xs2)
    mid = n // 2
    return float(xs2[mid]) if (n % 2 == 1) else float(0.5 * (xs2[mid-1] + xs2[mid]))


# ================================================================
# 방향(yaw) 및 벡터
# ================================================================
def _maybe_flip_yaw_right(px: float, split_x: float, yaw: float) -> float:
    if not C1_FLIP_RIGHT_SIDE:
        return _wrap_pi(yaw)
    if px > split_x:
        return _wrap_pi(yaw + math.pi)
    return _wrap_pi(yaw)


def _front_vec_from_yaw(yaw: float) -> tuple[float, float]:
    # map yaw 기준: yaw=0 → +Y front
    return (-math.sin(yaw), math.cos(yaw))


def _lateral_vec_from_yaw(yaw: float) -> tuple[float, float]:
    # map yaw 기준: left = (-cos, -sin)
    return (-math.cos(yaw), -math.sin(yaw))


def _clamp01(t: float) -> float:
    return 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)


def _project_point_to_segment(px, py, ax, ay, bx, by):
    """점 P를 선분 AB에 투영한 점 Q"""
    vx, vy = (bx - ax), (by - ay)
    wx, wy = (px - ax), (py - ay)
    vv = vx*vx + vy*vy
    if vv < 1e-9:
        return ax, ay, 0.0
    t = (wx*vx + wy*vy) / vv
    t = _clamp01(t)
    qx = ax + t*vx
    qy = ay + t*vy
    return qx, qy, t


def _yaw_from_linestring_tangent(line: SLineString, px: float, py: float) -> float:
    """
    LineString에서 (px,py)에 가장 가까운 위치의 접선 방향(rad, +x 기준)
    """
    P = SPoint(px, py)
    L = float(line.length)
    s = float(line.project(P))
    ds = min(2.0, 0.02 * L)
    s0 = max(0.0, s - ds)
    s1 = min(L,  s + ds)
    p0 = line.interpolate(s0)
    p1 = line.interpolate(s1)
    return math.atan2(p1.y - p0.y, p1.x - p0.x)


# ----------------------- C1 traffic lights --------------------------
def _quat_from_yaw_ros(yaw: float):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    return q


def _yaw_map_to_ros(yaw_map: float) -> float:
    # map yaw(0=+Y front) -> ros yaw(0=+X front)
    return _wrap_pi(yaw_map + math.pi/2.0)


def _quat_from_yaw_map(yaw: float) -> Quaternion:
    return _quat_from_yaw_ros(_yaw_map_to_ros(yaw))


def _yaw_box_from_front(yaw_front: float) -> float:
    # 박스의 local y축(scale.y)이 전방을 향하도록 local x축을 -90deg 회전
    return _wrap_pi(yaw_front - math.pi/2.0)


def _c1_type_spec(t: Optional[int]) -> dict:
    if t is None:
        return dict(name="unknown", orient="v", lamps=3, colors=[TL_COL_R, TL_COL_Y, TL_COL_G], kind="etc")

    if t == 1:
        return dict(name="veh_h_3", orient="h", lamps=3, colors=[TL_COL_R, TL_COL_Y, TL_COL_G], kind="vehicle")
    if t in (2, 3):
        return dict(name="veh_h_4", orient="h", lamps=4, colors=[TL_COL_R, TL_COL_Y, TL_COL_G, TL_COL_W], kind="vehicle")
    if t == 4:
        return dict(name="veh_h_arrow_3", orient="h", lamps=4, colors=[TL_COL_R, TL_COL_Y, TL_COL_G, TL_COL_W], kind="vehicle")
    if t == 5:
        return dict(name="veh_v_3", orient="v", lamps=3, colors=[TL_COL_R, TL_COL_Y, TL_COL_G], kind="vehicle")
    if t == 6:
        return dict(name="veh_v_arrow_3", orient="v", lamps=4, colors=[TL_COL_R, TL_COL_Y, TL_COL_G, TL_COL_W], kind="vehicle")
    if t == 7:
        return dict(name="veh_v_4", orient="v", lamps=4, colors=[TL_COL_R, TL_COL_Y, TL_COL_G, TL_COL_W], kind="vehicle")
    if t == 8:
        return dict(name="bus_3", orient="v", lamps=3, colors=[TL_COL_R, TL_COL_Y, TL_COL_G], kind="vehicle")
    if t in (9, 10):
        return dict(name="variable", orient="v", lamps=3, colors=[TL_COL_R, TL_COL_Y, TL_COL_G], kind="vehicle")
    if t == 11:
        return dict(name="ped_2", orient="v", lamps=2, colors=[TL_COL_R, TL_COL_G], kind="ped")
    if t == 12:
        return dict(name="bike_3", orient="v", lamps=3, colors=[TL_COL_R, TL_COL_Y, TL_COL_G], kind="bike")
    if t == 13:
        return dict(name="bike_2", orient="v", lamps=2, colors=[TL_COL_R, TL_COL_G], kind="bike")
    return dict(name=f"etc_{t}", orient="v", lamps=0, colors=[], kind="etc")


def _add_c1_head_only(ma: MarkerArray, ns: str, mid: int,
                      px: float, py: float, yaw_front: float,
                      t: Optional[int],
                      pole_h: float = TL_POLE_H) -> int:
    """
    C1 신호등 헤드(박스 + 램프)만 생성.
    - (px, py): head 위치 (map 좌표)
    - yaw_front: head가 바라보는 방향(map, +Y front)
    - t: Type 코드에 따라 orient/lamps/colors 결정
    - pole_h: 박스 z 위치 계산용 pole 높이(보행자 등 낮은 경우 조정)
    """
    spec = _c1_type_spec(t)

    # 박스는 전방에서 -90deg 회전한 yaw_box로 배치
    yaw_box = _yaw_box_from_front(yaw_front)
    q_box = _quat_from_yaw_map(yaw_box)

    if spec["kind"] == "etc":
        box = Marker(type=Marker.CUBE)
        box.scale.x = box.scale.y = 0.6
        box.scale.z = 0.8
        box.pose.position.x = px
        box.pose.position.y = py
        box.pose.position.z = 0.4
        box.pose.orientation = q_box
        ma.markers.append(HDMapPublisher._finish(box, ns, mid, _rgba(0.2, 0.8, 0.2, 1.0))); mid += 1
        return mid

    orient = spec["orient"]
    lamps  = int(spec["lamps"])

    if orient == "v":
        box_x = TL_BOX_W_V
        box_y = TL_BOX_D
        box_z = TL_BOX_H_V_4 if lamps >= 4 else TL_BOX_H_V_3
    else:
        box_x = TL_BOX_W_H_4 if lamps >= 4 else TL_BOX_W_H_3
        box_y = TL_BOX_D
        box_z = TL_BOX_H_H

    # 박스 생성
    box = Marker(type=Marker.CUBE)
    box.scale.x = box_x
    box.scale.y = box_y
    box.scale.z = box_z
    box.pose.position.x = px
    box.pose.position.y = py
    box.pose.position.z = pole_h - (box_z * 0.5)
    box.pose.orientation = q_box
    ma.markers.append(HDMapPublisher._finish(box, ns, mid, TL_COL_BOX)); mid += 1

    # 램프는 전방 기준 앞/뒤 면에 모두 생성(양면)
    # 기존은 (box_y*0.5 + 0.01)로 밖으로 배치했으나, face를 안쪽으로 살짝 넣는다.
    face = max(0.0, (box_y * 0.5) - TL_LAMP_INSET)

    cy = math.cos(yaw_front); sy = math.sin(yaw_front)
    # map yaw 기준 전방 벡터 = (-sin, cos)
    dirx = -sy
    diry =  cy

    # 앞/뒷면 오프셋 (대칭)
    fx_f = dirx * (face + TL_LAMP_EPS)
    fy_f = diry * (face + TL_LAMP_EPS)
    fx_b = dirx * (-(face + TL_LAMP_EPS))
    fy_b = diry * (-(face + TL_LAMP_EPS))

    colors = spec["colors"]

    if lamps > 0:
        if orient == "v":
            cz = pole_h - (box_z * 0.5)
            if lamps == 2:
                zs = [cz + 0.18, cz - 0.18]
            elif lamps == 3:
                zs = [cz + TL_LAMP_GAP, cz, cz - TL_LAMP_GAP]
            else:
                zs = [cz + 1.5*TL_LAMP_GAP, cz + 0.5*TL_LAMP_GAP, cz - 0.5*TL_LAMP_GAP, cz - 1.5*TL_LAMP_GAP]

            for lz, c in zip(zs, colors):
                # front
                lamp = Marker(type=Marker.SPHERE)
                lamp.scale.x = lamp.scale.y = lamp.scale.z = 2.0 * TL_LAMP_R
                lamp.pose.position.x = px + fx_f
                lamp.pose.position.y = py + fy_f
                lamp.pose.position.z = float(lz)
                lamp.pose.orientation.w = 1.0
                ma.markers.append(HDMapPublisher._finish(lamp, ns, mid, c)); mid += 1

                # back
                lamp = Marker(type=Marker.SPHERE)
                lamp.scale.x = lamp.scale.y = lamp.scale.z = 2.0 * TL_LAMP_R
                lamp.pose.position.x = px + fx_b
                lamp.pose.position.y = py + fy_b
                lamp.pose.position.z = float(lz)
                lamp.pose.orientation.w = 1.0
                ma.markers.append(HDMapPublisher._finish(lamp, ns, mid, c)); mid += 1

        else:
            # 수평형은 램프를 측방(left) 방향으로 배열
            lx, ly = _lateral_vec_from_yaw(yaw_front)  # left
            z0 = pole_h - (box_z * 0.5)
            if lamps == 3:
                offs = [-TL_LAMP_GAP, 0.0, +TL_LAMP_GAP]
            else:
                offs = [-1.5*TL_LAMP_GAP, -0.5*TL_LAMP_GAP, +0.5*TL_LAMP_GAP, +1.5*TL_LAMP_GAP]

            for o, c in zip(offs, colors):
                # front
                lamp = Marker(type=Marker.SPHERE)
                lamp.scale.x = lamp.scale.y = lamp.scale.z = 2.0 * TL_LAMP_R
                lamp.pose.position.x = px + fx_f + lx * o
                lamp.pose.position.y = py + fy_f + ly * o
                lamp.pose.position.z = float(z0)
                lamp.pose.orientation.w = 1.0
                ma.markers.append(HDMapPublisher._finish(lamp, ns, mid, c)); mid += 1

                # back
                lamp = Marker(type=Marker.SPHERE)
                lamp.scale.x = lamp.scale.y = lamp.scale.z = 2.0 * TL_LAMP_R
                lamp.pose.position.x = px + fx_b + lx * o
                lamp.pose.position.y = py + fy_b + ly * o
                lamp.pose.position.z = float(z0)
                lamp.pose.orientation.w = 1.0
                ma.markers.append(HDMapPublisher._finish(lamp, ns, mid, c)); mid += 1

    return mid


def _add_c1_mastarm_twoheads(ma: MarkerArray, ns: str, mid: int,
                             p1: tuple[float, float], p2: tuple[float, float],
                             yaw: float, t: Optional[int]) -> int:
    """
    Mast-arm 2헤드 버전(legacy): 두 좌표(p1,p2)를 pole+arm+head로 렌더.
    현재는 _gdf2markers에서 C6 기반 mastarm 처리로 주로 사용.
    """
    x1, y1 = p1
    x2, y2 = p2

    latx, laty = -math.sin(yaw), math.cos(yaw)
    proj1 = x1*latx + y1*laty
    proj2 = x2*latx + y2*laty
    if proj2 < proj1:
        x1, y1, x2, y2 = x2, y2, x1, y1

    q_pole = _quat_from_yaw_map(yaw)
    pole = Marker(type=Marker.CYLINDER)
    pole.scale.x = pole.scale.y = 2.0 * TL_POLE_R
    pole.scale.z = TL_POLE_H
    pole.pose.position.x = x1
    pole.pose.position.y = y1
    pole.pose.position.z = TL_POLE_H * 0.5
    pole.pose.orientation = q_pole
    ma.markers.append(HDMapPublisher._finish(pole, ns, mid, TL_COL_POLE)); mid += 1

    dx, dy = (x2 - x1), (y2 - y1)
    arm_len = math.hypot(dx, dy)
    arm_yaw = math.atan2(dy, dx)  # ROS yaw(+X 전면)
    q_arm = _quat_from_yaw_ros(arm_yaw)

    arm = Marker(type=Marker.CUBE)
    arm.scale.x = max(0.2, arm_len)
    arm.scale.y = TL_ARM_THICK
    arm.scale.z = TL_ARM_THICK
    arm.pose.position.x = (x1 + x2) * 0.5
    arm.pose.position.y = (y1 + y2) * 0.5
    arm.pose.position.z = TL_POLE_H - TL_ARM_DROP_Z
    arm.pose.orientation = q_arm
    ma.markers.append(HDMapPublisher._finish(arm, ns, mid, TL_COL_POLE)); mid += 1

    mid = _add_c1_head_only(ma, ns, mid, x1, y1, yaw, t)
    mid = _add_c1_head_only(ma, ns, mid, x2, y2, yaw, t)
    return mid


def _add_c1_traffic_light_markers(ma: MarkerArray, ns: str, mid: int,
                                  px: float, py: float, yaw: float,
                                  t: Optional[int]) -> int:
    """
    C1 단일 포스트.
    - (px, py): pole 위치(기본은 head와 동일)
    - yaw: map(front)
    - t: Type 코드
    """
    spec = _c1_type_spec(t)
    pole_h = TL_POLE_H - (TL_PED_LOWER if spec.get("kind") == "ped" else 0.0)
    # etc는 head_only로 처리
    if spec["kind"] == "etc":
        return _add_c1_head_only(ma, ns, mid, px, py, yaw, t, pole_h=pole_h)

    # pole은 공통
    q = _quat_from_yaw_map(yaw)
    pole = Marker(type=Marker.CYLINDER)
    pole.scale.x = pole.scale.y = 2.0 * TL_POLE_R
    pole.scale.z = pole_h
    pole.pose.position.x = px
    pole.pose.position.y = py
    pole.pose.position.z = pole_h * 0.5
    pole.pose.orientation = q
    ma.markers.append(HDMapPublisher._finish(pole, ns, mid, TL_COL_POLE)); mid += 1

    # head(박스+램프)는 head_only로 통일 (여기에 양면 램프 로직이 들어있음)
    return _add_c1_head_only(ma, ns, mid, px, py, yaw, t, pole_h=pole_h)


def _add_c1_traffic_light_markers_v2(ma: MarkerArray, ns: str, mid: int,
                                     head_xy: tuple[float, float],
                                     pole_xy: Optional[tuple[float, float]],
                                     yaw: float,
                                     t: Optional[int]) -> int:
    """
    C1 포스트+헤드 렌더러(v2): pole/head 위치를 분리 입력.
    - head_xy: head 위치 (필수)
    - pole_xy: pole 위치 (None이면 head 위치 재사용)
    - yaw: map
    - t: Type 코드
    """
    hx, hy = head_xy
    spec = _c1_type_spec(t)
    pole_h = TL_POLE_H - (TL_PED_LOWER if spec.get("kind") == "ped" else 0.0)

    # pole 위치 결정 (없으면 head에 세움)
    if pole_xy is None:
        px, py = hx, hy
    else:
        px, py = pole_xy

    # pole
    pole = Marker(type=Marker.CYLINDER)
    pole.scale.x = pole.scale.y = 2.0 * TL_POLE_R
    pole.scale.z = pole_h
    pole.pose.position.x = px
    pole.pose.position.y = py
    pole.pose.position.z = pole_h * 0.5
    pole.pose.orientation.w = 1.0
    ma.markers.append(HDMapPublisher._finish(pole, ns, mid, TL_COL_POLE)); mid += 1

    # arm (pole top -> head)
    arm = Marker(type=Marker.LINE_STRIP)
    arm.scale.x = TL_ARM_THICK
    arm.points = [
        Point(x=float(px), y=float(py), z=float(pole_h - TL_ARM_DROP_Z)),
        Point(x=float(hx), y=float(hy), z=float(pole_h - TL_ARM_DROP_Z)),
    ]
    ma.markers.append(HDMapPublisher._finish(arm, ns, mid, TL_COL_POLE)); mid += 1

    # head (box+lamps)
    mid = _add_c1_head_only(ma, ns, mid, hx, hy, yaw, t, pole_h=pole_h)
    return mid

# ----------------------- B2 dash --------------------------
def _parse_b2_type(type_val) -> tuple[ColorRGBA, float, int, bool]:
    """
    B2 Type 코드 → (색상, 폭배수, 패턴, 역방향) 매핑.
    - pattern: 0=solid, 1=dash(1:1), 2=dash(2:1) 등
    - 역방향(True)이면 라인 표기를 반대방향으로 그려야 할 때 사용
    """
    try:
        s = str(type_val).strip()
        if s == "" or s.lower() == "nan":
            return (B2_COL_OTHER, 1.0, 1, False)
        code = int(float(s))
    except Exception:
        return (B2_COL_OTHER, 1.0, 1, False)

    color_code = code // 100
    line_code  = (code // 10) % 10
    pattern    = code % 10

    if color_code == 1:
        rgba = B2_COL_YELLOW
    elif color_code == 2:
        rgba = B2_COL_WHITE
    elif color_code == 3:
        rgba = B2_COL_BLUE
    else:
        rgba = B2_COL_OTHER

    width_mul = 1.35 if line_code == 2 else 1.0
    return (rgba, width_mul, pattern, True)

def _is_dashed_pattern(pattern: int) -> bool:
    return pattern in (2, 3, 4)

def _make_dashed_line_list_points(line: SLineString,
                                  dash_len: float = 2.0,
                                  gap_len: float = 1.5,
                                  sample_step: float = 0.5,
                                  z: float = 0.02):
    pts: List[Point] = []
    L = float(line.length)
    if L < 1e-3:
        return pts

    period = dash_len + gap_len
    s = 0.0
    while s < L:
        s0 = s
        s1 = min(s + dash_len, L)

        ss = s0
        prev = line.interpolate(ss)
        ss += sample_step
        while ss <= s1 + 1e-6:
            cur = line.interpolate(min(ss, s1))
            pts.append(Point(x=float(prev.x), y=float(prev.y), z=z))
            pts.append(Point(x=float(cur.x),  y=float(cur.y),  z=z))
            prev = cur
            ss += sample_step

        s += period
    return pts


# ----------------------- C3 prism utils ---------------------------
def _resample_linestring(line: SLineString, step: float) -> list[tuple[float, float]]:
    """LineString을 step(m) 간격으로 리샘플한 (x,y) 리스트 반환."""
    L = float(line.length)
    if L < 1e-6:
        return []
    if L <= step:
        return [(float(x), float(y)) for (x, y) in list(line.coords)]
    n = int(L / step)
    pts = []
    for i in range(n + 1):
        s = min(i * step, L)
        p = line.interpolate(s)
        pts.append((float(p.x), float(p.y)))
    if pts:
        lx, ly = line.coords[-1]
        if abs(pts[-1][0] - lx) > 1e-6 or abs(pts[-1][1] - ly) > 1e-6:
            pts.append((float(lx), float(ly)))
    return pts


def _append_prism_segment_triangles(tris: list[Point],
                                    p0: tuple[float, float], p1: tuple[float, float],
                                    half_w: float, z0: float, z1: float):
    x0, y0 = p0
    x1, y1 = p1
    dx, dy = (x1 - x0), (y1 - y0)
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return

    nx, ny = (-dy / L), (dx / L)  # 좌측 법선

    l0b = (x0 + nx * half_w, y0 + ny * half_w, z0)
    r0b = (x0 - nx * half_w, y0 - ny * half_w, z0)
    l1b = (x1 + nx * half_w, y1 + ny * half_w, z0)
    r1b = (x1 - nx * half_w, y1 - ny * half_w, z0)

    l0t = (l0b[0], l0b[1], z1)
    r0t = (r0b[0], r0b[1], z1)
    l1t = (l1b[0], l1b[1], z1)
    r1t = (r1b[0], r1b[1], z1)

    def P(a):
        return Point(x=float(a[0]), y=float(a[1]), z=float(a[2]))

    tris.extend([P(l0b), P(l1b), P(l1t)])
    tris.extend([P(l0b), P(l1t), P(l0t)])

    tris.extend([P(r0b), P(r1t), P(r1b)])
    tris.extend([P(r0b), P(r0t), P(r1t)])

    tris.extend([P(l0t), P(r0t), P(r1t)])
    tris.extend([P(l0t), P(r1t), P(l1t)])


# ----------------------- Polygon utils --------------------
def _shift_polygon_keep_holes(p: Polygon, xo: float, yo: float) -> Polygon:
    # shp 폴리곤이 (x,y,z)일 수 있으니 앞 2개만 사용
    ext = [(c[0] - xo, c[1] - yo) for c in p.exterior.coords]
    holes = [[(c[0] - xo, c[1] - yo) for c in ring.coords] for ring in p.interiors]
    return Polygon(ext, holes)

def _triangulate_to_points(poly: Polygon, z: float = 0.01) -> List[Point]:
    pts: List[Point] = []
    if poly.is_empty or poly.area < 1e-6:
        return pts
    try:
        tris = triangulate(poly)
    except Exception:
        return pts

    for tri in tris:
        try:
            tri2 = tri.intersection(poly)
        except Exception:
            continue
        if tri2.is_empty:
            continue

        def add_one(p: Polygon):
            coords = list(p.exterior.coords)
            if len(coords) < 4:
                return
            a,b,c = coords[0], coords[1], coords[2]
            pts.append(Point(x=float(a[0]), y=float(a[1]), z=z))
            pts.append(Point(x=float(b[0]), y=float(b[1]), z=z))
            pts.append(Point(x=float(c[0]), y=float(c[1]), z=z))

        if isinstance(tri2, Polygon):
            add_one(tri2)
        elif isinstance(tri2, MultiPolygon):
            for g in tri2.geoms:
                add_one(g)

    return pts

def _mrr_pose(poly: Polygon) -> Tuple[float, float, float, float, float]:
    rect = poly.minimum_rotated_rectangle
    rc = list(rect.exterior.coords)
    if len(rc) < 4:
        c = poly.centroid
        return float(c.x), float(c.y), 0.0, 4.0, 1.6

    best = None
    for i in range(4):
        x1,y1 = rc[i][0], rc[i][1]
        x2,y2 = rc[i+1][0], rc[i+1][1]
        dx,dy = x2-x1, y2-y1
        L = math.hypot(dx,dy)
        if best is None or L > best[0]:
            best = (L, dx, dy)

    long_L, dx, dy = best
    if long_L < 1e-6:
        c = poly.centroid
        return float(c.x), float(c.y), 0.0, 4.0, 1.6

    yaw = math.atan2(dy, dx)
    rect_area = rect.area
    short_L = rect_area / long_L if long_L > 1e-6 else 1.6

    c = rect.centroid
    return float(c.x), float(c.y), float(yaw), float(long_L), float(short_L)

def _dominant_pose(poly: Polygon) -> Tuple[float, float, float, float, float]:
    c = poly.centroid
    coords = list(poly.exterior.coords)
    if len(coords) < 3:
        return float(c.x), float(c.y), 0.0, 4.0, 1.6

    sx = sy = 0.0
    for (x1, y1), (x2, y2) in zip(coords[:-1], coords[1:]):
        dx = x2 - x1
        dy = y2 - y1
        L  = math.hypot(dx, dy)
        if L < 1e-6:
            continue
        ang = math.atan2(dy, dx)
        sx += L * math.cos(2.0 * ang)
        sy += L * math.sin(2.0 * ang)

    if (sx * sx + sy * sy) < 1e-12:
        yaw = 0.0
    else:
        yaw = 0.5 * math.atan2(sy, sx)

    ux, uy = math.cos(yaw), math.sin(yaw)
    vx, vy = -uy, ux

    s_vals: List[float] = []
    t_vals: List[float] = []
    for x, y in coords:
        dx = x - c.x
        dy = y - c.y
        s_vals.append(ux * dx + uy * dy)
        t_vals.append(vx * dx + vy * dy)

    long_L  = max(s_vals) - min(s_vals)
    short_L = max(t_vals) - min(t_vals)
    long_L  = max(float(long_L),  0.5)
    short_L = max(float(short_L), 0.3)

    return float(c.x), float(c.y), float(yaw), float(long_L), float(short_L)


def _pca_pose(poly: Polygon) -> Tuple[float, float, float, float, float]:
    if poly.is_empty or poly.area < 1e-8:
        c = poly.centroid
        return float(c.x), float(c.y), 0.0, 4.0, 1.6

    coords = np.asarray(poly.exterior.coords, dtype=float)
    if coords.shape[0] < 3:
        c = poly.centroid
        return float(c.x), float(c.y), 0.0, 4.0, 1.6

    c = poly.centroid
    X = coords[:, :2] - np.array([[c.x, c.y]])

    # covariance + eigen
    C = np.cov(X.T)
    vals, vecs = np.linalg.eigh(C)
    v = vecs[:, int(np.argmax(vals))]  # principal axis
    yaw = float(math.atan2(v[1], v[0]))

    # projected extents
    u = np.array([math.cos(yaw), math.sin(yaw)])
    w = np.array([-u[1], u[0]])
    s = X @ u
    t = X @ w
    long_L  = float(np.max(s) - np.min(s))
    short_L = float(np.max(t) - np.min(t))

    long_L  = max(long_L,  0.5)
    short_L = max(short_L, 0.3)

    return float(c.x), float(c.y), yaw, long_L, short_L

# ----------------------- Crosswalk / Arrow helpers -----------------
def _make_head_triangle(x: float, y: float, yaw: float, head_l: float, head_w: float) -> Polygon:
    cy = math.cos(yaw); sy = math.sin(yaw)
    tx, ty = x, y
    bx, by = x - cy * head_l, y - sy * head_l
    nx, ny = -sy, cy
    lx, ly = bx + nx * (head_w / 2.0), by + ny * (head_w / 2.0)
    rx, ry = bx - nx * (head_w / 2.0), by - ny * (head_w / 2.0)
    return Polygon([(lx, ly), (tx, ty), (rx, ry)])


def _bezier_points(p0, p1, p2, p3, n: int = 28):
    """Cubic Bezier 샘플링"""
    pts = []
    for i in range(n + 1):
        t = i / n
        u = 1.0 - t
        x = (u*u*u)*p0[0] + 3*(u*u)*t*p1[0] + 3*u*(t*t)*p2[0] + (t*t*t)*p3[0]
        y = (u*u*u)*p0[1] + 3*(u*u)*t*p1[1] + 3*u*(t*t)*p2[1] + (t*t*t)*p3[1]
        pts.append((x, y))
    return pts


def _head_from_polyline_end(pts, head_l: float, head_w: float) -> Polygon:
    if len(pts) < 2:
        x, y = pts[-1]
        return _make_head_triangle(x, y, 0.0, head_l=head_l, head_w=head_w)
    (x2, y2), (x3, y3) = pts[-2], pts[-1]
    yaw = math.atan2(y3 - y2, x3 - x2)
    return _make_head_triangle(x3, y3, yaw, head_l=head_l, head_w=head_w)


def _make_curved_arrow_polygon_local(L: float, body_w: float,
                                     turn: str,
                                     radius: float,
                                     theta_deg: float,
                                     head_l: float,
                                     head_w: float,
                                     n_arc: int = 20) -> Polygon:

    if turn == "uturn":
        theta_deg = max(theta_deg, 170.0)

    sign = +1.0 if turn in ("left", "uturn") else -1.0
    theta = math.radians(theta_deg)

    straight_len = max(0.35 * L, L - radius * theta)
    x0 = -L / 2.0
    x1 = x0 + straight_len

    pts = [(x0, 0.0), (x1, 0.0)]

    cx = x1
    cy = sign * radius
    ang0 = -sign * (math.pi / 2.0)
    ang1 = ang0 + sign * theta

    for i in range(1, n_arc + 1):
        a = ang0 + (ang1 - ang0) * (i / n_arc)
        px = cx + radius * math.cos(a)
        py = cy + radius * math.sin(a)
        pts.append((px, py))

    endx, endy = pts[-1]
    end_yaw = ang1 + sign * (math.pi / 2.0)

    body = SLineString(pts).buffer(body_w / 2.0, cap_style=2, join_style=1)

    # tip을 너무 멀리 빼지 않고 적당히 앞으로
    tipx = endx + math.cos(end_yaw) * (head_l * 0.60)
    tipy = endy + math.sin(end_yaw) * (head_l * 0.60)
    head = _make_head_triangle(tipx, tipy, end_yaw, head_l=head_l, head_w=head_w)

    try:
        merged = unary_union([body, head])
    except Exception:
        merged = body

    if isinstance(merged, MultiPolygon):
        merged = max(list(merged.geoms), key=lambda p: p.area, default=body)

    return merged


def _make_curved_arrow_polygon(cx: float, cy: float, base_yaw: float,
                               L: float, body_w: float, head_l: float, head_w: float,
                               turn: str) -> Polygon:
    radius = max(1.0, min(L * 0.55, 4.0))
    theta_deg = 90.0 if turn in ("left", "right") else 180.0
    local = _make_curved_arrow_polygon_local(L=L, body_w=body_w, turn=turn,
                                             radius=radius, theta_deg=theta_deg,
                                             head_l=head_l, head_w=head_w)
    world = shp_translate(shp_rotate(local, angle=math.degrees(base_yaw), origin=(0, 0)),
                          xoff=cx, yoff=cy)
    return world


def _make_straight_arrow_local(L: float, body_w: float, head_l: float, head_w: float) -> Polygon:
    bw = body_w / 2.0
    hw = head_w / 2.0
    bl = L - head_l
    pts = [
        (-L/2, -bw),
        (bl - L/2, -bw),
        (bl - L/2, -hw),
        (L/2, 0.0),
        (bl - L/2, hw),
        (bl - L/2, bw),
        (-L/2, bw),
    ]
    return Polygon(pts)

def _make_straight_plus_side_connected_local(L: float, body_w: float,
                                             head_l: float, head_w: float,
                                             turn: str, short_L: float) -> Polygon:
    sign = +1.0 if turn == "left" else -1.0

    shaft_end = (L/2.0) - head_l
    main_line = SLineString([(-L/2.0, 0.0), (shaft_end, 0.0)])
    main_body = main_line.buffer(body_w/2.0, cap_style=2, join_style=2)
    main_head = _make_head_triangle(L/2.0, 0.0, 0.0, head_l=head_l, head_w=head_w)

    attach_x = (-L / 2.0) + 0.58 * L
    arm_len  = min(0.55 * L, 0.75 * short_L)
    arm_w    = max(0.20, 0.75 * body_w)
    arm_line = SLineString([(attach_x, 0.0), (attach_x, sign * arm_len)])
    arm_body = arm_line.buffer(arm_w/2.0, cap_style=2, join_style=2)

    side_hl = max(0.65, 0.55 * head_l)
    side_hw = max(0.70, 0.85 * head_w)
    tipx, tipy = attach_x, sign * arm_len
    arm_head = _make_head_triangle(tipx, tipy, sign*(math.pi/2.0), head_l=side_hl, head_w=side_hw)

    out = unary_union([main_body, main_head, arm_body, arm_head])
    if isinstance(out, Polygon):
        return out
    if isinstance(out, MultiPolygon):
        return unary_union(list(out.geoms))
    return main_head

def _make_turn_only_Lshape_local(L: float, body_w: float,
                                 head_l: float, head_w: float,
                                 turn: str, short_L: float,
                                 n_arc: int = 24) -> Polygon:
    sign = +1.0 if turn == "left" else -1.0

    r = min(max(0.55 * body_w, 0.55), 0.45 * short_L, 0.35 * L)
    pre  = max(0.30 * L, L - (math.pi/2) * r - head_l*0.6)
    post = max(0.22 * L, 0.35 * short_L)

    x0 = -L/2.0
    x1 = x0 + pre
    pts = [(x0, 0.0), (x1, 0.0)]

    cx = x1
    cy = sign * r
    a0 = -sign * (math.pi/2.0)
    a1 = 0.0
    for i in range(1, n_arc + 1):
        a = a0 + (a1 - a0) * (i / n_arc)
        px = cx + r * math.cos(a)
        py = cy + r * math.sin(a)
        pts.append((px, py))

    endx, endy = pts[-1]
    pts.append((endx, endy + sign * post))

    body = SLineString(pts).buffer(body_w/2.0, cap_style=2, join_style=1)

    tipx, tipy = pts[-1]
    tip_yaw = sign * (math.pi/2.0)
    head = _make_head_triangle(tipx, tipy, tip_yaw, head_l=head_l, head_w=head_w)

    out = unary_union([body, head])
    if isinstance(out, Polygon):
        return out
    if isinstance(out, MultiPolygon):
        return unary_union(list(out.geoms))
    return body


def _make_turn_only_smooth_local(L: float, body_w: float,
                                 head_l: float, head_w: float,
                                 turn: str, short_L: float) -> Polygon:
    sign = +1.0 if turn == "left" else -1.0

    tail_len = 0.58 * L
    end_y    = sign * min(0.62 * short_L, 0.42 * L)
    end_x    = (-L/2.0) + 0.78 * L

    p0 = (-L/2.0, 0.0)
    pj = (p0[0] + tail_len, 0.0)

    p1 = (pj[0] + 0.22 * L, 0.0)
    p3 = (end_x, end_y)
    p2 = (p3[0], 0.55 * end_y)

    pts = [p0, pj] + _bezier_points(pj, p1, p2, p3, n=26)[1:]

    body = SLineString(pts).buffer(body_w/2.0, cap_style=2, join_style=1)
    head = _head_from_polyline_end(pts, head_l=head_l, head_w=head_w)

    out = unary_union([body, head])
    if isinstance(out, MultiPolygon):
        out = max(list(out.geoms), key=lambda g: g.area, default=body)
    return out

def _make_straight_plus_right_T_local(L: float, body_w: float,
                                      head_l: float, head_w: float,
                                      turn: str, short_L: float) -> Polygon:
    sign = +1.0 if turn == "left" else -1.0

    main = _make_straight_arrow_local(L, body_w, head_l, head_w)

    side_L  = max(1.8, 0.55 * L)
    side_bw = max(0.20, 0.80 * body_w)
    side_hw = max(0.70, 1.75 * side_bw)
    side_hl = max(0.70, 0.60 * head_l)

    side = _make_straight_arrow_local(side_L, side_bw, side_hl, side_hw)
    side = shp_rotate(side, angle=sign * 90.0, origin=(0.0, 0.0))

    attach_x = (-L/2.0) + 0.55 * L
    attach_y = sign * (0.55 * body_w)

    side = shp_translate(side, xoff=attach_x, yoff=attach_y)

    out = unary_union([main, side])
    if isinstance(out, Polygon):
        return out
    if isinstance(out, MultiPolygon):
        return unary_union(list(out.geoms))
    return main


def _cubic_bezier_points(p0, p1, p2, p3, n=30):
    """cubic Bezier를 n개 점으로 샘플링"""
    pts = []
    for i in range(n + 1):
        t = i / n
        u = 1.0 - t
        x = (u*u*u)*p0[0] + 3*(u*u)*t*p1[0] + 3*u*(t*t)*p2[0] + (t*t*t)*p3[0]
        y = (u*u*u)*p0[1] + 3*(u*u)*t*p1[1] + 3*u*(t*t)*p2[1] + (t*t*t)*p3[1]
        pts.append((x, y))
    return pts


def _make_split_straight_turn_local(L: float, body_w: float,
                                    head_l: float, head_w: float,
                                    turn: str, short_L: float,
                                    split_ratio: float = 0.56,
                                    branch_w_ratio: float = 0.92,
                                    branch_len_ratio: float = 0.85) -> Polygon:
    """
    직진+회전을 도로마킹처럼 분기형으로 생성.
    로컬좌표: 진행 +x, left +y, right -y
    """
    sign = +1.0 if turn == "left" else -1.0

    # 공통 stem
    x_tail = -L / 2.0
    x_head_base = (L / 2.0) - head_l
    main_center = SLineString([(x_tail, 0.0), (x_head_base, 0.0)])
    main_body = main_center.buffer(body_w / 2.0, cap_style=2, join_style=2)
    main_head = _make_head_triangle(L/2.0, 0.0, 0.0, head_l=head_l, head_w=head_w)

    # 분기 시작
    x_split = x_tail + split_ratio * L

    branch_end_y = sign * min(0.46 * short_L, branch_len_ratio * (L * 0.55))
    branch_end_x = x_split + min(0.22 * L, 0.55 * short_L)

    # Bezier로 가지 곡선 생성
    p0 = (x_split, 0.0)
    p1 = (x_split + 0.16 * L, 0.0)
    p2 = (x_split + 0.18 * L, 0.55 * branch_end_y)
    p3 = (branch_end_x, branch_end_y)
    branch_pts = _cubic_bezier_points(p0, p1, p2, p3, n=34)
    branch_center = SLineString(branch_pts)

    branch_w = max(0.20, branch_w_ratio * body_w)
    branch_body = branch_center.buffer(branch_w / 2.0, cap_style=2, join_style=2)

    tip_yaw = sign * (math.pi / 2.0)
    tipx = p3[0] + math.cos(tip_yaw) * (0.55 * head_l)
    tipy = p3[1] + math.sin(tip_yaw) * (0.55 * head_l)
    branch_head_l = max(0.70, 0.60 * head_l)
    branch_head_w = max(0.80, 0.85 * head_w)
    branch_head = _make_head_triangle(tipx, tipy, tip_yaw, head_l=branch_head_l, head_w=branch_head_w)

    out = unary_union([main_body, main_head, branch_body, branch_head])
    if isinstance(out, Polygon):
        return out
    if isinstance(out, MultiPolygon):
        return max(list(out.geoms), key=lambda g: g.area, default=main_body)
    return main_body


def _make_straight_plus_turn_merged_local(L: float, body_w: float,
                                          head_l: float, head_w: float,
                                          turn: str, short_L: float) -> Polygon:
    """
    직진+좌/우 (실도로 느낌): 메인 직진 몸통 + 분기 곡선(베지어) + 분기 화살촉
    """
    sign = +1.0 if turn == "left" else -1.0

    main = _make_straight_arrow_local(L, body_w, head_l, head_w)

    attach_x = (-L/2.0) + 0.55 * L
    p0 = (attach_x, 0.0)

    end_y = sign * min(0.60 * short_L, 0.40 * L)
    end_x = (-L/2.0) + 0.78 * L
    p3 = (end_x, end_y)

    p1 = (p0[0] + 0.18 * L, 0.0)
    p2 = (p3[0], 0.55 * end_y)
    pts = _bezier_points(p0, p1, p2, p3, n=26)

    branch_w = max(0.20, 0.85 * body_w)
    branch_body = SLineString(pts).buffer(branch_w/2.0, cap_style=2, join_style=1)

    bh_l = max(0.70, 0.80 * head_l)
    bh_w = max(0.70, 0.85 * head_w)
    branch_head = _head_from_polyline_end(pts, head_l=bh_l, head_w=bh_w)

    out = unary_union([main, branch_body, branch_head])
    if isinstance(out, Polygon):
        return out
    if isinstance(out, MultiPolygon):
        out = max(list(out.geoms), key=lambda g: g.area, default=main)
    return out


def _make_turn_arrow_local(L: float, body_w: float, head_l: float, head_w: float,
                           turn: str, short_L: float) -> Polygon:
    margin = 0.35 * body_w + 0.30 * head_w
    r_max = max(0.75, 0.50 * short_L - margin)
    radius = min(max(0.95, 0.42 * L), r_max)
    theta_deg = 90.0 if turn in ("left", "right") else 180.0

    return _make_curved_arrow_polygon_local(
        L=L, body_w=body_w, turn=turn,
        radius=radius, theta_deg=theta_deg,
        head_l=head_l, head_w=head_w,
        n_arc=26
    )


def _make_branch_only_local(L: float, body_w: float, head_l: float, head_w: float,
                            turn: str, short_L: float) -> Polygon:
    sign = +1.0 if turn == "left" else -1.0

    branch_w = max(0.70 * body_w, 0.38)

    x_attach = 0.10 * L
    pre_len  = min(0.22 * L, 0.55 * short_L)
    post_len = min(0.40 * L, 0.60 * short_L)

    p0 = (x_attach, 0.0)
    p1 = (x_attach + pre_len, 0.0)
    p2 = (x_attach + pre_len, sign * post_len)

    stem = SLineString([p0, p1, p2]).buffer(branch_w / 2.0, cap_style=2, join_style=2)

    side_head_w = max(1.60 * branch_w, 0.55 * head_w)
    side_head_l = max(1.05 * side_head_w, 0.60 * head_l)

    tipx, tipy = p2[0], p2[1] + sign * (0.55 * side_head_l)
    side_yaw = sign * (math.pi / 2.0)
    head = _make_head_triangle(tipx, tipy, side_yaw, head_l=side_head_l, head_w=side_head_w)

    out = unary_union([stem, head])
    if isinstance(out, Polygon):
        return out
    if isinstance(out, MultiPolygon):
        try:
            return unary_union(list(out.geoms))
        except Exception:
            return out
    return stem


def _make_fork_arrow_local(L: float, body_w: float, head_l: float, head_w: float,
                           turn: str, short_L: float) -> Polygon:
    """
    직진 + (좌/우) 를 'T 분기' 형태로 생성 (메인+팔+머리 별도)
    """
    main = _make_straight_arrow_local(L, body_w, head_l, head_w)

    sign = +1.0 if turn == "left" else -1.0

    shaft_end = (L / 2.0) - head_l
    shaft_len = shaft_end - (-L / 2.0)

    attach_x = (-L / 2.0) + FORK_ATTACH_RATIO * shaft_len
    pre_len  = FORK_PRE_RATIO * L
    arm_len  = min(FORK_ARM_RATIO * L, 0.48 * short_L)

    path = SLineString([
        (attach_x, 0.0),
        (attach_x + pre_len, 0.0),
        (attach_x + pre_len, sign * arm_len),
    ])

    branch_w = max(0.20, FORK_BRANCH_W_RATIO * body_w)
    branch_body = path.buffer(branch_w / 2.0, cap_style=2, join_style=1)

    bh_l = max(0.75 * head_l, 0.75)
    bh_w = max(0.95 * head_w, 2.1 * branch_w)

    tipx = attach_x + pre_len
    tipy = sign * (arm_len + 0.60 * bh_l)
    byaw = sign * (math.pi / 2.0)
    branch_head = _make_head_triangle(tipx, tipy, byaw, head_l=bh_l, head_w=bh_w)

    out = unary_union([main, branch_body, branch_head])
    if isinstance(out, Polygon):
        return out
    if isinstance(out, MultiPolygon):
        try:
            return unary_union(list(out.geoms))
        except Exception:
            return out
    return main


def _make_arrow_poly_world(cx: float, cy: float, yaw: float,
                           L: float, body_w: float, head_l: float, head_w: float,
                           kind_dir: str, short_L: float) -> Polygon:
    if kind_dir == "straight":
        p = _make_straight_arrow_local(L, body_w, head_l, head_w)
    elif kind_dir == "left":
        p = _make_turn_arrow_local(L, body_w, head_l, head_w, turn="left", short_L=short_L)
    elif kind_dir == "right":
        p = _make_turn_arrow_local(L, body_w, head_l, head_w, turn="right", short_L=short_L)
    elif kind_dir == "fork_left":
        p = _make_fork_arrow_local(L, body_w, head_l, head_w, turn="left", short_L=short_L)
    elif kind_dir == "fork_right":
        p = _make_fork_arrow_local(L, body_w, head_l, head_w, turn="right", short_L=short_L)
    else:
        p = _make_straight_arrow_local(L, body_w, head_l, head_w)

    p = shp_rotate(p, yaw * 180.0 / math.pi, origin=(0.0, 0.0))
    p = shp_translate(p, xoff=cx, yoff=cy)
    return p


def _flip_yaw_to_head(poly: Polygon, yaw: float) -> float:
    ux, uy = math.cos(yaw), math.sin(yaw)
    vx, vy = -uy, ux

    coords = list(poly.exterior.coords)
    if len(coords) < 4:
        return yaw

    s_vals = [ux*x + uy*y for x, y in coords]
    s_min, s_max = min(s_vals), max(s_vals)
    L = s_max - s_min
    if L < 1e-3:
        return yaw

    w = 0.15 * L

    def width_in_window(s0, s1):
        vs = []
        for (x, y), s in zip(coords, s_vals):
            if s0 <= s <= s1:
                vs.append(vx*x + vy*y)
        return (max(vs) - min(vs)) if len(vs) >= 2 else 0.0

    w_min = width_in_window(s_min, s_min + w)
    w_max = width_in_window(s_max - w, s_max)

    if w_min > w_max:
        yaw += math.pi
    return yaw


def _kind_to_dirs(kind: Optional[int]) -> List[str]:
    """
    반환: ["straight"], ["right"], ["straight","right"] 등 복수 방향 지원
    """
    if kind is None:
        return ["straight"]

    mapping = {
        5371: ["straight"],
        5372: ["left"],
        5373: ["right"],
        5374: ["left", "right"],
        5379: ["straight", "left", "right"],
        5381: ["straight", "left"],
        5382: ["straight", "right"],
        5383: ["straight", "uturn"],
        5391: ["uturn"],
        5392: ["left", "uturn"],
        5431: ["left"],
        5432: ["right"],
    }
    return mapping.get(kind, ["straight"])


def _looks_like_stripe_multipolygon(mp: MultiPolygon) -> bool:
    geoms = list(mp.geoms)
    if len(geoms) >= 8:
        areas = [g.area for g in geoms if g.area > 1e-6]
        if not areas:
            return False
        total = sum(areas)
        avg = total / len(areas)
        return avg < (total * 0.15)
    return False


def _crosswalk_stripes_from_polygon(poly: Polygon) -> List[Polygon]:
    cx, cy, yaw, long_L, short_L = _mrr_pose(poly)

    stripe_w = 0.65
    gap_w    = 0.45
    if short_L < 2.0:
        stripe_w, gap_w = 0.45, 0.30

    ux, uy = math.cos(yaw), math.sin(yaw)       # long-axis unit
    vx, vy = -uy, ux                            # short-axis unit

    # 스트라이프를 short-axis 방향으로 길게, long-axis로 이동
    pitch = stripe_w + gap_w
    n = max(2, int(long_L / pitch) + 1)

    start = -long_L / 2.0 + stripe_w / 2.0
    half_short = short_L / 2.0 + 0.2

    stripes: List[Polygon] = []
    for i in range(n):
        t = start + i * pitch
        scx = cx + ux * t
        scy = cy + uy * t
        half_sw = stripe_w / 2.0

        p1 = (scx - vx * half_short - ux * half_sw, scy - vy * half_short - uy * half_sw)
        p2 = (scx + vx * half_short - ux * half_sw, scy + vy * half_short - uy * half_sw)
        p3 = (scx + vx * half_short + ux * half_sw, scy + vy * half_short + uy * half_sw)
        p4 = (scx - vx * half_short + ux * half_sw, scy - vy * half_short + uy * half_sw)
        stripe = Polygon([p1, p2, p3, p4])

        try:
            clipped = stripe.intersection(poly)
        except Exception:
            continue
        if clipped.is_empty:
            continue

        if isinstance(clipped, Polygon):
            if clipped.area > 1e-4:
                stripes.append(clipped)
        else:
            for g in clipped.geoms:
                if g.area > 1e-4:
                    stripes.append(g)

    return stripes


def _fit_into_footprint(ap: Polygon, footprint: Polygon, short_L: float,
                        yaw: float, long_L: float) -> Polygon:
    """
    footprint 안으로 최대한 안 짤리게 넣기: tail 기준 스케일 탐색 + 클리핑
    """
    if ap.is_empty:
        return ap

    inset = max(0.01, 0.010 * short_L)
    clip = footprint.buffer(-inset)
    if clip.is_empty:
        clip = footprint

    u = (math.cos(yaw), math.sin(yaw))
    c = footprint.centroid
    tailx = c.x - u[0] * (0.35 * long_L)
    taily = c.y - u[1] * (0.35 * long_L)
    origin = (tailx, taily)

    best = None
    for s in (1.00, 0.96, 0.92, 0.88, 0.84, 0.80, 0.76, 0.72):
        ap2 = shp_scale(ap, xfact=s, yfact=s, origin=origin)
        try:
            inter = ap2.intersection(clip)
        except Exception:
            inter = ap2

        if inter.is_empty:
            continue
        if ap2.area < 1e-9:
            continue

        ratio = inter.area / ap2.area
        if best is None or ratio > best[0]:
            best = (ratio, inter)
        if ratio >= 0.97:
            return inter

    if best is not None:
        return best[1]

    try:
        return ap.intersection(footprint)
    except Exception:
        return ap

def _clean_poly(g):
    if g is None or g.is_empty:
        return g
    try:
        g = g.buffer(0)
    except Exception:
        pass
    if isinstance(g, MultiPolygon):
        geoms = [x for x in g.geoms if (not x.is_empty and x.area > 1e-6)]
        if geoms:
            try:
                g = unary_union(geoms)
            except Exception:
                g = MultiPolygon(geoms)
    try:
        g = g.simplify(0.02, preserve_topology=True)
    except Exception:
        pass
    return g


def _arrow_polys_from_footprint(cx: float, cy: float, yaw: float,
                                long_L: float, short_L: float,
                                dirs: List[str]) -> List[Polygon]:
    eff_long  = 0.90 * long_L
    eff_short = 0.86 * short_L

    L = max(ARROW_L_MIN, min(eff_long * ARROW_L_RATIO, ARROW_L_MAX))
    body_w = max(ARROW_BODY_W_MIN, min(eff_short * ARROW_BODY_W_RATIO, ARROW_BODY_W_MAX))

    head_l = max(ARROW_HEAD_L_MIN, min(L * ARROW_HEAD_L_RATIO, ARROW_HEAD_L_MAX))
    head_w = max(ARROW_HEAD_W_MIN, min(body_w * ARROW_HEAD_W_RATIO, ARROW_HEAD_W_MAX))
    head_w = min(head_w, 0.80 * eff_short)

    ds = list(dict.fromkeys(dirs))
    dset = set(ds)

    if dset == {"straight", "right"}:
        local = _make_straight_plus_turn_merged_local(L, body_w, head_l, head_w, turn="right", short_L=short_L)
    elif dset == {"straight", "left"}:
        local = _make_straight_plus_turn_merged_local(L, body_w, head_l, head_w, turn="left",  short_L=short_L)
    elif dset == {"straight", "left", "right"}:
        local = unary_union([
            _make_split_straight_turn_local(L, body_w, head_l, head_w, turn="left",  short_L=short_L),
            _make_split_straight_turn_local(L, body_w, head_l, head_w, turn="right", short_L=short_L),
        ])
    elif dset == {"right"}:
        local = _make_turn_only_smooth_local(L, body_w, head_l, head_w, turn="right", short_L=short_L)
    elif dset == {"left"}:
        local = _make_turn_only_smooth_local(L, body_w, head_l, head_w, turn="left",  short_L=short_L)
    else:
        parts = []
        for d in ds:
            if d == "straight":
                parts.append(_make_straight_arrow_local(L, body_w, head_l, head_w))
            elif d == "left":
                parts.append(_make_turn_arrow_local(L, body_w, head_l, head_w, turn="left",  short_L=short_L))
            elif d == "right":
                parts.append(_make_turn_arrow_local(L, body_w, head_l, head_w, turn="right", short_L=short_L))
            elif d == "uturn":
                parts.append(_make_turn_arrow_local(L, body_w, head_l, head_w, turn="uturn", short_L=short_L))
            else:
                parts.append(_make_straight_arrow_local(L, body_w, head_l, head_w))

        try:
            local = unary_union(parts)
        except Exception:
            local = parts[0]
        if isinstance(local, MultiPolygon):
            local = max(list(local.geoms), key=lambda g: g.area, default=parts[0])

    world = shp_translate(
        shp_rotate(local, angle=math.degrees(yaw), origin=(0, 0)),
        xoff=cx, yoff=cy
    )
    return [world]

# ─────────────────────────────────────────────────────────
class HDMapPublisher(Node):
    """HD 맵을 *.shp에서 읽어 RViz MarkerArray로 퍼블리시하는 노드."""
    def __init__(self):
        super().__init__("hd_map_publisher")
        # get shp map path
        self.declare_parameter("map_path", "/home/aiict/KIAPI_ioniq5/map_data")
        input_path_str = self.get_parameter("map_path").value
        self.data_root = Path(input_path_str) / "KIAPI_SHP"
        self.get_logger().info(f"[Init] Target Data Root: {self.data_root}")

        self._pubs : Dict[str, rclpy.publisher.Publisher] = {}
        self._cache: Dict[str, MarkerArray]               = {}
        self._x_off = self._y_off = 0.0
        self._a2_links: Dict[str, SLineString]            = {}
        self._a2_lines: List[SLineString]                 = []
        self._a2_ids:   List[str]                         = []
        self._a2_tree = None
        self._a2_geom2idx = {}
        self._c6_posts: Dict[str, tuple[float, float, float]] = {}
        self._c6_pts: List[SPoint]                        = []
        self._c6_ids: List[str]                           = []
        self._c6_tree = None
        self._c6_geom2idx = {}

        self._decide_offset()
        self._broadcast_tf()
        self._load_a2_link_index()
        self._load_c6_post_index()
        self._scan_load()
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        
    def _decide_offset(self):
        if X_OFFSET is not None and Y_OFFSET is not None:
            self._x_off = float(X_OFFSET)
            self._y_off = float(Y_OFFSET)
            try:
                OFF_YAML.write_text(yaml.safe_dump({"x_off": self._x_off, "y_off": self._y_off}))
                self.get_logger().info(f"[Offset] Fixed  ({self._x_off:.1f}, {self._y_off:.1f})  → offset.yaml 저장")
            except Exception as e:
                self.get_logger().warn(f"[Offset] Fixed 저장 실패: {e}")
            return

        if OFF_YAML.exists():
            try:
                d = yaml.safe_load(OFF_YAML.read_text()) or {}
                self._x_off, self._y_off = float(d["x_off"]), float(d["y_off"])
                self.get_logger().info(f"[Offset] Loaded ({self._x_off:.1f}, {self._y_off:.1f})")
                return
            except Exception as e:
                self.get_logger().warn(f"offset.yaml 파싱 실패({e}) → 재계산")

        #shp_any = next(DATA_ROOT.rglob("*.shp"), None)
        shp_any = next(self.data_root.rglob("*.shp"), None)
        if shp_any is None:
            self.get_logger().fatal("*.shp 파일을 찾을 수 없습니다.")
            rclpy.shutdown()
            return
        minx, miny, *_ = gpd.read_file(shp_any, columns=[]).total_bounds
        self._x_off, self._y_off = float(minx), float(miny)
        OFF_YAML.write_text(yaml.safe_dump({"x_off": self._x_off, "y_off": self._y_off}))
        self.get_logger().info(f"[Offset] Auto   ({self._x_off:.1f}, {self._y_off:.1f})  → offset.yaml 저장")

    def _broadcast_tf(self):
        tfb = tf2_ros.StaticTransformBroadcaster(self)
        tf  = TransformStamped()
        tf.header.stamp     = self.get_clock().now().to_msg()
        tf.header.frame_id  = "world"
        tf.child_frame_id   = FRAME_ID
        tf.transform.translation.x = -self._x_off
        tf.transform.translation.y = -self._y_off
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w    = 1.0
        tfb.sendTransform(tf)
        self.get_logger().info(f"[TF] world → {FRAME_ID}  (-{self._x_off:.1f}, -{self._y_off:.1f}, 0)")

    def _load_a2_link_index(self):
        #shp = next((p for p in DATA_ROOT.rglob("*.shp") if p.stem.lower() == "a2_link"), None)
        shp = next((p for p in self.data_root.rglob("*.shp") if p.stem.lower() == "a2_link"), None)
        if shp is None:
            self.get_logger().warning("[A2] A2_LINK.shp not found → C1 yaw uses heading fallback")
            return

        gdf = self._safe_read(shp, need_attrs=True)
        if gdf is None or len(gdf) == 0:
            self.get_logger().warning("[A2] A2_LINK read failed/empty")
            return

        id_col = _colname_case_insensitive(gdf, "ID")
        if id_col is None:
            self.get_logger().warning("[A2] ID column not found in A2_LINK")
            return

        xo, yo = self._x_off, self._y_off
        n_ok = 0
        for _, r in gdf.iterrows():
            lid = str(r.get(id_col, "")).strip()
            if not lid or lid.lower() == "nan":
                continue
            geom = r.geometry
            if geom is None:
                continue

            if geom.geom_type == "MultiLineString":
                lines = list(geom.geoms)
                geom = max(lines, key=lambda x: x.length, default=None)
                if geom is None:
                    continue
            if geom.geom_type not in ("LineString", "LinearRing"):
                continue

            coords = [(p[0] - xo, p[1] - yo) for p in geom.coords]
            line = SLineString(coords)
            if line.length < 1e-3:
                continue

            self._a2_links[lid] = line
            n_ok += 1

        from shapely.strtree import STRtree

        self._a2_ids = list(self._a2_links.keys())
        self._a2_lines = [self._a2_links[k] for k in self._a2_ids]
        if self._a2_lines:
            self._a2_tree = STRtree(self._a2_lines)
            self._a2_geom2idx = {id(g): i for i, g in enumerate(self._a2_lines)}
        else:
            self._a2_tree = None
            self._a2_geom2idx = {}

        self.get_logger().info(f"[A2] link index built: {n_ok}")

    def _load_c6_post_index(self):
        #shp = next((p for p in DATA_ROOT.rglob("*.shp") if p.stem.lower() == "c6_postpoint"), None)
        shp = next((p for p in self.data_root.rglob("*.shp") if p.stem.lower() == "c6_postpoint"), None)
        if shp is None:
            self.get_logger().warning("[C6] C6_POSTPOINT.shp not found → pole uses C1 point fallback")
            return

        gdf = self._safe_read(shp, need_attrs=True)
        if gdf is None or len(gdf) == 0:
            self.get_logger().warning("[C6] read failed/empty")
            return

        id_col = _colname_case_insensitive(gdf, "ID")
        if id_col is None:
            self.get_logger().warning("[C6] ID column not found")
            return

        xo, yo = self._x_off, self._y_off
        posts = {}
        pts = []
        ids = []
        for _, r in gdf.iterrows():
            pid = str(r.get(id_col, "")).strip()
            g = r.geometry
            if not pid or pid.lower() == "nan":
                continue
            if g is None or g.geom_type != "Point":
                continue
            px = float(g.x - xo)
            py = float(g.y - yo)
            pz = float(getattr(g, "z", 0.0) if hasattr(g, "z") else 0.0)
            posts[pid] = (px, py, pz)
            pts.append(SPoint(px, py))
            ids.append(pid)

        from shapely.strtree import STRtree
        self._c6_posts = posts
        self._c6_pts = pts
        self._c6_ids = ids
        if pts:
            self._c6_tree = STRtree(pts)
            self._c6_geom2idx = {id(g): i for i, g in enumerate(pts)}
            self.get_logger().info(f"[C6] post index built: {len(pts)}")
        else:
            self._c6_tree = None
            self._c6_geom2idx = {}
            self.get_logger().warning("[C6] no valid points")

    def _safe_read(self, shp: Path, need_attrs: bool = False):
        try:
            if not need_attrs:
                return gpd.read_file(shp, columns=[], engine="fiona")
            try:
                return gpd.read_file(shp, engine="fiona")
            except UnicodeDecodeError:
                for enc in ("cp949", "euc-kr", "utf-8"):
                    try:
                        return gpd.read_file(shp, engine="fiona", encoding=enc)
                    except Exception:
                        pass
                self.get_logger().warning(f" Attr read fail → geometry only fallback  {shp.name}")
                return gpd.read_file(shp, columns=[], engine="fiona")
        except Exception as e:
            self.get_logger().warning(f" Read fail → skip  {shp.name} ({e})")
        return None

    def _nearest_a2_tangent_yaw(self, px: float, py: float, search_r: float = 8.0):
        """
        (px,py)에서 search_r 안에 있는 A2_LINK 중 가장 가까운 라인의 접선 yaw를 반환.
        반환 yaw는 "map yaw(0=+Y front)" 기준으로 변환해서 줌.
        """
        if not self._a2_links:
            return 0.0, None, float("inf")

        P = SPoint(float(px), float(py))
        best = None  # (d, lid, line_geom)

        for lid, line in self._a2_links.items():
            # line이 shapely geometry인지 강제 체크
            if line is None or not hasattr(line, "distance") or not hasattr(line, "bounds"):
                continue

            minx, miny, maxx, maxy = line.bounds
            if px < (minx - search_r) or px > (maxx + search_r) or py < (miny - search_r) or py > (maxy + search_r):
                continue

            d = float(P.distance(line))
            if d <= search_r and (best is None or d < best[0]):
                best = (d, lid, line)

        if best is None:
            return 0.0, None, float("inf")

        d, lid, line = best
        yaw_ros = _yaw_from_linestring_tangent(line, px, py)  # atan2(dy,dx), ros yaw
        yaw_map = _wrap_pi(yaw_ros - math.pi/2.0)             # ros -> map
        return yaw_map, lid, float(d)

    def _nearest_a2_dist(self, px: float, py: float, search_r: float = 20.0) -> float:
        """A2 링크까지의 최소 거리(없으면 inf)."""
        if not self._a2_links:
            return float("inf")
        P = SPoint(float(px), float(py))
        best = float("inf")
        for _, line in self._a2_links.items():
            if line is None or not hasattr(line, "distance") or not hasattr(line, "bounds"):
                continue
            minx, miny, maxx, maxy = line.bounds
            if px < (minx - search_r) or px > (maxx + search_r) or py < (miny - search_r) or py > (maxy + search_r):
                continue
            d = float(P.distance(line))
            if d < best:
                best = d
        return best

    def _a5_synthesize_slots(self, p2: Polygon, n: int) -> list[Polygon]:
        """
        A5 폴리곤 내부에 슬롯 n개를 합성 생성(사각형들).
        - MRR yaw로 로컬 정렬 후 long축 등분, short축 한쪽에서 depth만큼 생성
        - depth/폭에 최소/최대/ratio 제약 적용 후 원 폴리곤으로 클리핑
        """
        if p2.is_empty or p2.area < 1e-6:
            return []

        cx, cy, yaw, long_L, short_L = _mrr_pose(p2)  # yaw: ros(+x) 기준
        p_local = shp_rotate(p2, -math.degrees(yaw), origin=(cx, cy))
        minx, miny, maxx, maxy = p_local.bounds
        Lx = maxx - minx
        Ly = maxy - miny
        if Lx < 3.0 or Ly < 2.0:
            return []

        depth = A5_SLOT_DEPTH_RATIO * Ly
        depth = max(A5_SLOT_MIN_D, min(depth, A5_SLOT_MAX_D))
        depth = min(depth, Ly - 2.0 * A5_SLOT_MARGIN)
        if depth <= 0.5:
            return []

        ux, uy = math.cos(yaw), math.sin(yaw)
        vx, vy = -uy, ux
        test_off = 0.45 * Ly
        p_neg = (cx - vx * test_off, cy - vy * test_off)  # local miny side
        p_pos = (cx + vx * test_off, cy + vy * test_off)  # local maxy side
        d_neg = self._nearest_a2_dist(p_neg[0], p_neg[1], search_r=25.0)
        d_pos = self._nearest_a2_dist(p_pos[0], p_pos[1], search_r=25.0)
        use_miny_side = True if (d_neg == float("inf") and d_pos == float("inf")) else (d_neg <= d_pos)

        if use_miny_side:
            y0 = miny + A5_SLOT_MARGIN
            y1 = y0 + depth
        else:
            y1 = maxy - A5_SLOT_MARGIN
            y0 = y1 - depth

        usable = Lx - 2.0 * A5_SLOT_MARGIN - (n - 1) * A5_SLOT_GAP
        if usable <= n * A5_SLOT_MIN_W:
            gap2 = max(0.10, 0.5 * A5_SLOT_GAP)
            usable = Lx - 2.0 * A5_SLOT_MARGIN - (n - 1) * gap2
            if usable <= n * A5_SLOT_MIN_W:
                return []
            gap = gap2
        else:
            gap = A5_SLOT_GAP

        slot_w = usable / n
        slot_w = max(A5_SLOT_MIN_W, slot_w)

        clip = p2.buffer(-0.02)
        if clip.is_empty:
            clip = p2

        slots: list[Polygon] = []
        x = minx + A5_SLOT_MARGIN
        for _ in range(n):
            x0 = x
            x1 = x0 + slot_w
            rect_local = Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])

            rect_world = shp_rotate(rect_local, math.degrees(yaw), origin=(cx, cy))
            try:
                rect_world = rect_world.intersection(clip)
            except Exception:
                pass

            if rect_world.is_empty:
                x = x1 + gap
                continue

            if isinstance(rect_world, Polygon):
                if rect_world.area > 0.15:
                    slots.append(rect_world)
            elif isinstance(rect_world, MultiPolygon):
                for g in rect_world.geoms:
                    if g.area > 0.15:
                        slots.append(g)

            x = x1 + gap

        return slots

    def _c1_pole_from_post(self, row, c1_post_col: Optional[str],
                           px: float, py: float,
                           search_r: float = 10.0) -> tuple[Optional[float], Optional[float], Optional[str], float]:
        """
        postID 우선 → 없으면 nearest post.
        returns: (pole_x, pole_y, post_id, dist)
        Shapely 2.x STRtree 반환이 index일 수 있으므로 대응
        """
        P = SPoint(px, py)

        # 1) attr postID 우선
        post_id = None
        if c1_post_col is not None:
            v = row.get(c1_post_col, None)
            if v is not None:
                s = str(v).strip()
                if s and s.lower() not in ("nan", "none"):
                    post_id = s

        if post_id is not None and post_id in self._c6_posts:
            val = self._c6_posts[post_id]
            if isinstance(val, SPoint):
                px0, py0 = float(val.x), float(val.y)
                d = float(P.distance(val))
            else:
                try:
                    px0, py0 = float(val[0]), float(val[1])
                except Exception:
                    return (None, None, None, float("inf"))
                d = float(math.hypot(px - px0, py - py0))
            return (px0, py0, post_id, d)

        # 2) nearest post fallback
        if self._c6_tree is None or not self._c6_pts:
            return (None, None, None, float("inf"))

        def _as_index(x) -> Optional[int]:
            if x is None:
                return None
            if isinstance(x, (int, np.integer)):
                i = int(x)
                return i if 0 <= i < len(self._c6_pts) else None
            try:
                i = self._c6_geom2idx.get(id(x), None)
                return int(i) if i is not None else None
            except Exception:
                return None

        def _geom_at(i: int):
            return self._c6_pts[i]

        idx = None
        try:
            idx = _as_index(self._c6_tree.nearest(P))
        except Exception:
            idx = None

        if idx is None:
            env = P.buffer(search_r).envelope
            try:
                cands = self._c6_tree.query(env)
            except Exception:
                cands = []

            best_i = None
            best_d = float("inf")
            for c in cands:
                ci = _as_index(c)
                if ci is None:
                    continue
                g = _geom_at(ci)
                d = float(P.distance(g))
                if d < best_d:
                    best_d = d
                    best_i = ci
            idx = best_i

        if idx is None:
            return (None, None, None, float("inf"))

        geom = _geom_at(idx)
        d = float(P.distance(geom))
        if d > search_r:
            return (None, None, None, d)

        pid = self._c6_ids[idx] if 0 <= idx < len(self._c6_ids) else None
        return (float(geom.x), float(geom.y), pid, d)

    def _scan_load(self):
        """DATA_ROOT 이하 *.shp를 스캔해 레이어별 토픽으로 퍼블리셔/캐시를 구성."""
        #region = slug(DATA_ROOT.name)
        #all_shp: List[Path] = list(DATA_ROOT.rglob("*.shp"))
        region = slug(self.data_root.name)
        all_shp: List[Path] = list(self.data_root.rglob("*.shp"))
        if not all_shp:
            self.get_logger().error("*.shp 파일이 없습니다.")
            return

        for idx, shp in enumerate(all_shp):
            zone  = slug(shp.parent.parent.name) if shp.parent.parent else "zone"
            layer = slug(shp.stem)
            topic = f"/{region}/{zone}/{layer}"
            color = COLOR10[idx % len(COLOR10)]

            stem_l = shp.stem.lower()
            is_b2 = "b2_surfacelinemark" in stem_l
            is_b3 = "b3_surfacemark" in stem_l  # 이번 파일명에 정확히 맞춤
            is_c1 = (("c1" in stem_l and ("signal" in stem_l or "traffic" in stem_l)) or ("c1_" in stem_l))
            is_c3 = ("c3" in stem_l) and ("vehicle" in stem_l) and ("protection" in stem_l)
            is_a5 = ("a5_parkinglot" in stem_l) or (stem_l.startswith("a5") and "parking" in stem_l)
            
            need_attrs = (is_b2 or is_b3 or is_c1 or is_c3 or is_a5)
            gdf = self._safe_read(shp, need_attrs=need_attrs)
            if gdf is None:
                continue
            mk  = self._gdf2markers(gdf, layer, color, is_b2=is_b2, is_b3=is_b3, is_c1=is_c1, is_c3=is_c3, is_a5=is_a5)
            self._pubs[topic]  = self.create_publisher(MarkerArray, topic, 10)
            self._cache[topic] = mk

            #rel = str(shp.relative_to(DATA_ROOT))
            rel = str(shp.relative_to(self.data_root))
            tag = "B2" if is_b2 else ("B3" if is_b3 else ("A5" if is_a5 else ("C3" if is_c3 else "--")))
            self.get_logger().info(f"[{tag}] {rel:<60} → {len(mk.markers):4d} ▶ {topic}")

    def _gdf2markers(self, gdf, ns: str, col: ColorRGBA,
                     is_b2: bool = False, is_b3: bool = False,
                     is_c1: bool = False, is_c3: bool = False,
                     is_a5: bool = False) -> MarkerArray:
        """
        GeoDataFrame → MarkerArray 변환 엔트리포인트.
        - ns: topic/layer 이름
        - col: 기본 색 (B2/B3는 type/kind에 따라 덮어씀)
        - is_b2 / is_b3 / is_c1 / is_c3 / is_a5: 레이어 타입 플래그
        주요 분기:
          * C1: 포스트/헤드 매칭, mastarm 생성
          * C3: vehicle protection safety (연석/가드레일/방호벽) 3D 블록
          * A5: parking lot(외곽선/옵션 fill)
          * B3: 횡단보도/화살표/기타 폴리곤
          * B2/기타: 라인/포인트 기본 렌더
        """
        ma = MarkerArray(); mid = 0
        xo, yo = self._x_off, self._y_off

        # 컬럼
        b2_type_col = _colname_case_insensitive(gdf, "Type") if is_b2 else None
        b3_type_col = _colname_case_insensitive(gdf, "Type") if is_b3 else None
        b3_kind_col = _colname_case_insensitive(gdf, "Kind") if is_b3 else None
        b3_head_col = _find_heading_col(gdf) if is_b3 else None
        c3_type_col      = _colname_case_insensitive(gdf, "Type") if is_c3 else None
        c3_iscentral_col = _colname_case_insensitive(gdf, "IsCentral") if is_c3 else None
        a5_type_col = _colname_case_insensitive(gdf, "Type") if is_a5 else None
        if is_c1:
            # ---- column detect ----
            c1_type_col = _colname_case_insensitive(gdf, "Type")
            c1_post_col = _colname_case_insensitive(gdf, "postID")
            c1_id_col   = _colname_case_insensitive(gdf, "ID")

            # ---- collect items ----
            items = []
            for _, r in gdf.iterrows():
                g = r.geometry
                if g is None or g.geom_type != "Point":
                    continue

                tl_id = str(r.get(c1_id_col, "")).strip() if c1_id_col else ""
                t_c1  = _get_int(r, c1_type_col)

                # C1 점(대부분 head 위치로 보는 게 자연스럽습니다)
                hx = float(g.x - xo); hy = float(g.y - yo)
                hz = float(getattr(g, "z", 0.0)) if hasattr(g, "z") else 0.0

                # postID 우선
                post_id = str(r.get(c1_post_col, "")).strip() if c1_post_col else ""
                base = None
                if post_id and post_id.lower() != "nan" and post_id in self._c6_posts:
                    base = self._c6_posts[post_id]

                # postID가 없거나 누락이면 nearest post로 백업(55개라 brute-force로 충분)
                if base is None and self._c6_posts:
                    bestp = None
                    for pid, (px, py, pz) in self._c6_posts.items():
                        d = math.hypot(hx - px, hy - py)
                        if d <= POST_ASSOC_MAX and (bestp is None or d < bestp[0]):
                            bestp = (d, pid, (px, py, pz))
                    if bestp is not None:
                        post_id = bestp[1]
                        base = bestp[2]

                # yaw는 LinkID(속성)는 신뢰 불가 판정이었으므로, A2 tangent로만 추정
                yaw_map, lid_phys, d_a2 = self._nearest_a2_tangent_yaw(hx, hy, search_r=A2_YAW_SEARCH_R)

                spec = _c1_type_spec(t_c1)
                yaw_use = yaw_map
                if spec.get("kind") == "ped":
                    yaw_use = _wrap_pi(yaw_use + TL_PED_YAW_OFFSET)
                items.append({
                    "tl_id": tl_id,
                    "t": t_c1,
                    "kind": spec.get("kind", "etc"),
                    "hx": hx, "hy": hy, "hz": hz,
                    "post_id": post_id,
                    "base": base,           # (bx,by,bz) or None
                    "yaw": yaw_use,         # map yaw (ped는 오프셋 적용)
                    "a2": (lid_phys, d_a2),
                })

            # ---- grouping by post_id (mastarm 후보는 같은 post 공유) ----
            by_post: Dict[str, list] = {}
            singles_no_post = []
            for it in items:
                if it["base"] is None or not it["post_id"]:
                    singles_no_post.append(it)
                    continue
                by_post.setdefault(it["post_id"], []).append(it)

            # ---- render: posts with items ----
            for pid, group in by_post.items():
                bx, by, bz = group[0]["base"]
                # 같은 post 내에서도 kind=vehicle만 mastarm 후보
                veh = [x for x in group if x["kind"] == "vehicle"]
                nonveh = [x for x in group if x["kind"] != "vehicle"]

                # non-vehicle (ped/bike/etc) 는 개별 pole+head로
                for it in nonveh:
                    mid = _add_c1_traffic_light_markers(ma, ns, mid, bx, by, it["yaw"], it["t"])

                # vehicle 그룹 처리
                if len(veh) < 2:
                    # 1개면: pole은 post, head는 C1 좌표에 찍되 pole은 공통 위치로
                    if len(veh) == 1:
                        it = veh[0]
                        # pole
                        q = _quat_from_yaw_map(it["yaw"])
                        pole = Marker(type=Marker.CYLINDER)
                        pole.scale.x = pole.scale.y = 2.0 * TL_POLE_R
                        pole.scale.z = TL_POLE_H
                        pole.pose.position.x = bx
                        pole.pose.position.y = by
                        pole.pose.position.z = TL_POLE_H * 0.5
                        pole.pose.orientation = q
                        ma.markers.append(self._finish(pole, ns, mid, TL_COL_POLE)); mid += 1

                        # arm: post -> head
                        hx, hy = float(it["hx"]), float(it["hy"])
                        dx = hx - bx
                        dy = hy - by
                        Larm = math.hypot(dx, dy)
                        arm_z = TL_POLE_H - TL_ARM_DROP_Z

                        if Larm >= 0.20:
                            arm_yaw_ros = math.atan2(dy, dx)
                            q_arm = _quat_from_yaw_ros(arm_yaw_ros)

                            arm = Marker(type=Marker.CUBE)
                            arm.scale.x = max(0.2, Larm)
                            arm.scale.y = TL_ARM_THICK
                            arm.scale.z = TL_ARM_THICK
                            arm.pose.position.x = bx + 0.5 * dx
                            arm.pose.position.y = by + 0.5 * dy
                            arm.pose.position.z = arm_z
                            arm.pose.orientation = q_arm
                            ma.markers.append(self._finish(arm, ns, mid, TL_COL_POLE)); mid += 1

                        # head는 C1 점 위치(arm이 있든 없든 head는 이게 더 자연스러움)
                        mid = _add_c1_head_only(ma, ns, mid, hx, hy, it["yaw"], it["t"])
                    continue

                # ---- mastarm(2-head) 선택: lateral 조건 + yaw 조건 ----
                # yaw 기준 front/lat 분해해서 "거의 lat 방향으로 벌어진 쌍"을 찾음
                best_pair = None  # (score, i, j)
                if len(veh) == 2:
                    a, b = veh[0], veh[1]
                    yaw = _yaw_mean(a["yaw"], b["yaw"])
                    fx, fy = _front_vec_from_yaw(yaw)
                    lx, ly = _lateral_vec_from_yaw(yaw)
                    dx = b["hx"] - a["hx"]; dy = b["hy"] - a["hy"]
                    d  = math.hypot(dx, dy)
                    lat = abs(dx*lx + dy*ly)
                    fr  = abs(dx*fx + dy*fy)
                    self.get_logger().info(
                        f"[C1] pid={pid} veh2 check: d={d:.2f}, lat={lat:.2f}, fr={fr:.2f}, "
                        f"lat_ratio={lat/max(d,1e-6):.2f}, yawdiff={math.degrees(_angdist(a['yaw'], b['yaw'])):.1f}deg"
                    )
                for i in range(len(veh)):
                    for j in range(i+1, len(veh)):
                        a, b = veh[i], veh[j]
                        yaw = _yaw_mean(a["yaw"], b["yaw"])
                        fx, fy = _front_vec_from_yaw(yaw)
                        lx, ly = _lateral_vec_from_yaw(yaw)

                        dx = b["hx"] - a["hx"]
                        dy = b["hy"] - a["hy"]
                        d  = math.hypot(dx, dy)
                        if not (C1_PAIR_DIST_MIN <= d <= C1_PAIR_DIST_MAX):
                            continue

                        lat = abs(dx*lx + dy*ly)
                        fr  = abs(dx*fx + dy*fy)

                        # lateral 지배 조건: lat >= ratio * d
                        if lat < (C1_PAIR_LATERAL_RATIO * d):
                            continue

                        # yaw 차이 제한(이미 각자 yaw가 A2로 맞춰져 있어서 크게 튀면 제외)
                        if _angdist(a["yaw"], b["yaw"]) > math.radians(C1_PAIR_YAW_MAX_DEG):
                            continue

                        # score: lat 크게, fr 작게 선호
                        score = lat - 0.5*fr
                        if best_pair is None or score > best_pair[0]:
                            best_pair = (score, i, j)

                if best_pair is None:
                    # ✅ veh가 2개인데 페어 실패 시 강제로 mastarm pair로 렌더
                    if len(veh) == 2:
                        h1, h2 = veh[0], veh[1]
                        x1, y1 = float(h1["hx"]), float(h1["hy"])
                        x2, y2 = float(h2["hx"]), float(h2["hy"])

                        span_yaw_ros = math.atan2(y2 - y1, x2 - x1)
                        span_yaw_map = _wrap_pi(span_yaw_ros - math.pi/2.0)
                        yaw_pair = _wrap_pi(span_yaw_map - math.pi/2.0)

                        self.get_logger().info(
                            f"[C1] best_pair NONE but veh=2 -> FORCE mastarm pair  pid={pid} "
                            f"(span_map={span_yaw_map:+.2f}, yaw_pair={yaw_pair:+.2f})"
                        )

                        q_pole = _quat_from_yaw_map(yaw_pair)
                        pole = Marker(type=Marker.CYLINDER)
                        pole.scale.x = pole.scale.y = 2.0 * TL_POLE_R
                        pole.scale.z = TL_POLE_H
                        pole.pose.position.x = bx
                        pole.pose.position.y = by
                        pole.pose.position.z = TL_POLE_H * 0.5
                        pole.pose.orientation = q_pole
                        ma.markers.append(self._finish(pole, ns, mid, TL_COL_POLE)); mid += 1

                        arm_z = TL_POLE_H - TL_ARM_DROP_Z

                        dx12, dy12 = (x2 - x1), (y2 - y1)
                        span_len = math.hypot(dx12, dy12)
                        if span_len >= 0.25:
                            arm_span_yaw_ros = math.atan2(dy12, dx12)
                            q_span = _quat_from_yaw_ros(arm_span_yaw_ros)

                            arm_span = Marker(type=Marker.CUBE)
                            arm_span.scale.x = max(0.2, span_len)
                            arm_span.scale.y = TL_ARM_THICK
                            arm_span.scale.z = TL_ARM_THICK
                            arm_span.pose.position.x = (x1 + x2) * 0.5
                            arm_span.pose.position.y = (y1 + y2) * 0.5
                            arm_span.pose.position.z = arm_z
                            arm_span.pose.orientation = q_span
                            ma.markers.append(self._finish(arm_span, ns, mid, TL_COL_POLE)); mid += 1

                        qx, qy, _ = _project_point_to_segment(bx, by, x1, y1, x2, y2)
                        dxp, dyp = (qx - bx), (qy - by)
                        attach_len = math.hypot(dxp, dyp)
                        if attach_len >= 0.20:
                            arm_attach_yaw_ros = math.atan2(dyp, dxp)
                            q_attach = _quat_from_yaw_ros(arm_attach_yaw_ros)

                            arm_attach = Marker(type=Marker.CUBE)
                            arm_attach.scale.x = max(0.2, attach_len)
                            arm_attach.scale.y = TL_ARM_THICK
                            arm_attach.scale.z = TL_ARM_THICK
                            arm_attach.pose.position.x = bx + 0.5 * dxp
                            arm_attach.pose.position.y = by + 0.5 * dyp
                            arm_attach.pose.position.z = arm_z
                            arm_attach.pose.orientation = q_attach
                            ma.markers.append(self._finish(arm_attach, ns, mid, TL_COL_POLE)); mid += 1

                        mid = _add_c1_head_only(ma, ns, mid, x1, y1, yaw_pair, h1["t"])
                        mid = _add_c1_head_only(ma, ns, mid, x2, y2, yaw_pair, h2["t"])
                        continue

                    # 기존 fallback: 개별 pole+head
                    for it in veh:
                        q = _quat_from_yaw_map(it["yaw"])
                        pole = Marker(type=Marker.CYLINDER)
                        pole.scale.x = pole.scale.y = 2.0 * TL_POLE_R
                        pole.scale.z = TL_POLE_H
                        pole.pose.position.x = bx
                        pole.pose.position.y = by
                        pole.pose.position.z = TL_POLE_H * 0.5
                        pole.pose.orientation = q
                        ma.markers.append(self._finish(pole, ns, mid, TL_COL_POLE)); mid += 1

                        mid = _add_c1_head_only(ma, ns, mid, it["hx"], it["hy"], it["yaw"], it["t"])
                    continue

                _, i, j = best_pair

                # heads
                h1, h2 = veh[i], veh[j]
                yaw_pair = _yaw_mean(h1["yaw"], h2["yaw"])

                # pole at post (그대로 유지)
                q_pole = _quat_from_yaw_map(yaw_pair)
                pole = Marker(type=Marker.CYLINDER)
                pole.scale.x = pole.scale.y = 2.0 * TL_POLE_R
                pole.scale.z = TL_POLE_H
                pole.pose.position.x = bx
                pole.pose.position.y = by
                pole.pose.position.z = TL_POLE_H * 0.5
                pole.pose.orientation = q_pole
                ma.markers.append(self._finish(pole, ns, mid, TL_COL_POLE)); mid += 1

                # ---- arm geometry: span between two heads ----
                x1, y1 = float(h1["hx"]), float(h1["hy"])
                x2, y2 = float(h2["hx"]), float(h2["hy"])

                # 가로보 높이
                arm_z = TL_POLE_H - TL_ARM_DROP_Z

                # (A) 가로보: head1 <-> head2
                dx12, dy12 = (x2 - x1), (y2 - y1)
                span_len = math.hypot(dx12, dy12)

                if span_len >= 0.25:
                    arm_span_yaw_ros = math.atan2(dy12, dx12)
                    q_span = _quat_from_yaw_ros(arm_span_yaw_ros)

                    arm_span = Marker(type=Marker.CUBE)
                    arm_span.scale.x = max(0.2, span_len)
                    arm_span.scale.y = TL_ARM_THICK
                    arm_span.scale.z = TL_ARM_THICK
                    arm_span.pose.position.x = (x1 + x2) * 0.5
                    arm_span.pose.position.y = (y1 + y2) * 0.5
                    arm_span.pose.position.z = arm_z
                    arm_span.pose.orientation = q_span
                    ma.markers.append(self._finish(arm_span, ns, mid, TL_COL_POLE)); mid += 1

                # (B) 폴대 -> 가로보 접점: pole을 head1-head2 선분에 투영한 점 q
                qx, qy, _ = _project_point_to_segment(bx, by, x1, y1, x2, y2)
                dxp, dyp = (qx - bx), (qy - by)
                attach_len = math.hypot(dxp, dyp)

                if attach_len >= 0.20:
                    arm_attach_yaw_ros = math.atan2(dyp, dxp)
                    q_attach = _quat_from_yaw_ros(arm_attach_yaw_ros)

                    arm_attach = Marker(type=Marker.CUBE)
                    arm_attach.scale.x = max(0.2, attach_len)
                    arm_attach.scale.y = TL_ARM_THICK
                    arm_attach.scale.z = TL_ARM_THICK
                    arm_attach.pose.position.x = bx + 0.5 * dxp
                    arm_attach.pose.position.y = by + 0.5 * dyp
                    arm_attach.pose.position.z = arm_z
                    arm_attach.pose.orientation = q_attach
                    ma.markers.append(self._finish(arm_attach, ns, mid, TL_COL_POLE)); mid += 1

                # heads at C1 points (박스+램프)
                mid = _add_c1_head_only(ma, ns, mid, x1, y1, yaw_pair, h1["t"])
                mid = _add_c1_head_only(ma, ns, mid, x2, y2, yaw_pair, h2["t"])

            # ---- items without any post ----
            for it in singles_no_post:
                # post를 못 찾았으면 어쩔 수 없이 C1 점 자체에 pole+head
                mid = _add_c1_traffic_light_markers(ma, ns, mid, it["hx"], it["hy"], it["yaw"], it["t"])

            return ma

        for _, row in gdf.iterrows():
            geom = row.geometry
            if geom is None:
                continue

            gt = geom.geom_type

            # ===================== C3 (Vehicle Protection Safety) =====================
            if is_c3:
                if gt not in ("LineString", "LinearRing", "MultiLineString", "Polygon", "MultiPolygon"):
                    continue

                t = _get_int(row, c3_type_col)
                st = C3_STYLE.get(t, C3_DEFAULT)
                if c3_type_col is not None:
                    self.get_logger().info(f"[C3] type={t}, geom={gt}")

                # ---- Type=99: Polygon(또는 닫힌 선) → 건물 박스처럼 표시 ----
                if t == 99:
                    polys = []
                    if gt == "Polygon":
                        polys = [geom]
                    elif gt == "MultiPolygon":
                        polys = list(geom.geoms)
                    if not polys and gt in ("LineString", "LinearRing"):
                        try:
                            coords0 = list(geom.coords)
                            if len(coords0) >= 4 and (coords0[0][0], coords0[0][1]) == (coords0[-1][0], coords0[-1][1]):
                                polys = [Polygon(coords0)]
                        except Exception:
                            polys = []

                    for p in polys:
                        p2 = _shift_polygon_keep_holes(p, xo, yo)
                        cx, cy, yaw_ros, long_L, short_L = _mrr_pose(p2)
                        h = float(st["h"])

                        box = Marker(type=Marker.CUBE)
                        box.scale.x = max(0.5, long_L)
                        box.scale.y = max(0.5, short_L)
                        box.scale.z = h
                        box.pose.position.x = cx
                        box.pose.position.y = cy
                        box.pose.position.z = float(C3_Z_BASE + 0.5*h)
                        box.pose.orientation = _quat_from_yaw_ros(yaw_ros)
                        ma.markers.append(self._finish(box, ns, mid, st["col"], alpha=st["a"])); mid += 1
                    continue

                if gt in ("Polygon", "MultiPolygon"):
                    continue

                lines = [geom] if gt != "MultiLineString" else list(geom.geoms)

                fill = Marker(type=Marker.TRIANGLE_LIST)
                fill.scale.x = fill.scale.y = fill.scale.z = 1.0

                top = Marker(type=Marker.LINE_LIST)
                top.scale.x = 0.06

                posts = None
                if C3_POST_ENABLE and st["name"] in ("rail", "barrier"):
                    posts = Marker(type=Marker.CUBE_LIST)
                    posts.scale.x = posts.scale.y = C3_POST_SIZE_W
                    posts.scale.z = C3_POST_SIZE_H
                    posts.pose.orientation.w = 1.0

                half_w = 0.5 * float(st["w"])
                z0 = float(C3_Z_BASE)
                z1 = float(C3_Z_BASE + st["h"])
                z_edge = z1 + max(0.01, float(C3_OUTLINE_Z_ADD))

                for ln_geom in lines:
                    coords = [(p[0]-xo, p[1]-yo) for p in ln_geom.coords]
                    line = SLineString(coords)
                    if float(line.length) < 1e-3:
                        continue

                    pts = _resample_linestring(line, step=C3_RESAMPLE_STEP)
                    if len(pts) < 2:
                        continue

                    for a, b in zip(pts[:-1], pts[1:]):
                        _append_prism_segment_triangles(fill.points, a, b, half_w=half_w, z0=z0, z1=z1)

                        x0, y0 = a; x1, y1 = b
                        dx, dy = (x1 - x0), (y1 - y0)
                        Lseg = math.hypot(dx, dy)
                        if Lseg > 1e-6:
                            nx, ny = (-dy / Lseg), (dx / Lseg)
                            lt0 = (x0 + nx*half_w, y0 + ny*half_w, z_edge)
                            lt1 = (x1 + nx*half_w, y1 + ny*half_w, z_edge)
                            rt0 = (x0 - nx*half_w, y0 - ny*half_w, z_edge)
                            rt1 = (x1 - nx*half_w, y1 - ny*half_w, z_edge)

                            top.points.append(Point(x=float(lt0[0]), y=float(lt0[1]), z=float(lt0[2])))
                            top.points.append(Point(x=float(lt1[0]), y=float(lt1[1]), z=float(lt1[2])))
                            top.points.append(Point(x=float(rt0[0]), y=float(rt0[1]), z=float(rt0[2])))
                            top.points.append(Point(x=float(rt1[0]), y=float(rt1[1]), z=float(rt1[2])))

                    if posts is not None:
                        acc = 0.0
                        prev = None
                        for (x, y) in pts:
                            if prev is None:
                                prev = (x, y)
                                posts.points.append(Point(x=float(x), y=float(y), z=float(z0 + 0.5*C3_POST_SIZE_H)))
                                continue
                            acc += math.hypot(x - prev[0], y - prev[1])
                            if acc >= C3_POST_EVERY_M:
                                posts.points.append(Point(x=float(x), y=float(y), z=float(z0 + 0.5*C3_POST_SIZE_H)))
                                acc = 0.0
                            prev = (x, y)

                if C3_OUTLINE_ENABLE and len(top.points) > 0:
                    ma.markers.append(self._finish(top, ns, mid, C3_OUTLINE_COL, alpha=1.0)); mid += 1
                ma.markers.append(self._finish(fill, ns, mid, st["col"], alpha=st["a"])); mid += 1
                if posts is not None and len(posts.points) > 0:
                    ma.markers.append(self._finish(posts, ns, mid, C3_POST_COL, alpha=1.0)); mid += 1
                continue

            # ===================== B3 =====================
            if is_b3:
                # 이번 B3는 Polygon만 들어옴(로그로 확인)
                if gt not in ("Polygon", "MultiPolygon"):
                    continue

                t = _get_int(row, b3_type_col)
                k = _get_int(row, b3_kind_col)

                polys = [geom] if gt == "Polygon" else list(geom.geoms)

                # (A) 횡단보도: Type=5 → stripe 채움(테두리 없음)
                if t == 5:
                    polys2 = [_shift_polygon_keep_holes(p, xo, yo) for p in polys]

                    fill = Marker(type=Marker.TRIANGLE_LIST)
                    fill.scale.x = fill.scale.y = fill.scale.z = 1.0

                    for p2 in polys2:
                        inset = max(0.05, 0.04 * math.sqrt(max(p2.area, 1e-6)))
                        p_in = p2.buffer(-inset)
                        if p_in.is_empty:
                            p_in = p2

                        if isinstance(p_in, Polygon):
                            stripes = _crosswalk_stripes_from_polygon(p_in)
                            if stripes:
                                for sp in stripes:
                                    fill.points.extend(_triangulate_to_points(sp, z=0.01))
                            else:
                                fill.points.extend(_triangulate_to_points(p_in, z=0.01))

                    ma.markers.append(self._finish(fill, ns, mid, B3_COL_WHITE, alpha=B3_FILL_ALPHA)); mid += 1
                    continue

                # (B) 화살표: Type=1 → Kind 기반 곡선/포크 화살표 생성
                if t == 1:
                    outline = Marker(type=Marker.LINE_LIST)
                    outline.scale.x = B3_OUTLINE_W

                    fill = Marker(type=Marker.TRIANGLE_LIST)
                    fill.scale.x = fill.scale.y = fill.scale.z = 1.0

                    dirs = _kind_to_dirs(k)

                    for p in polys:
                        p2 = _shift_polygon_keep_holes(p, xo, yo)

                        cx, cy, yaw_geom, long_L, short_L = _pca_pose(p2)

                        hv = _get_float(row, b3_head_col) if b3_head_col is not None else None
                        if hv is not None:
                            yaw = _yaw_from_heading_azimuth(hv, yaw_geom)
                        else:
                            yaw = yaw_geom
                            yaw = _direct_yaw_if_asymmetric(p2, yaw)

                        arrow_polys = _arrow_polys_from_footprint(cx, cy, yaw, long_L, short_L, dirs)

                        for ap in arrow_polys:
                            ap = _fit_into_footprint(ap, p2, short_L, yaw=yaw, long_L=long_L)
                            ap = _clean_poly(ap)
                            if ap.is_empty:
                                continue
                            ap_list = ap.geoms if isinstance(ap, MultiPolygon) else [ap]
                            for ap_poly in ap_list:
                                coords = list(ap_poly.exterior.coords)
                                for a, b in zip(coords[:-1], coords[1:]):
                                    outline.points.append(Point(x=float(a[0]), y=float(a[1]), z=0.02))
                                    outline.points.append(Point(x=float(b[0]), y=float(b[1]), z=0.02))
                                fill.points.extend(_triangulate_to_points(ap_poly, z=0.01))

                    ma.markers.append(self._finish(outline, ns, mid, B3_COL_WHITE, alpha=1.0)); mid += 1
                    ma.markers.append(self._finish(fill,    ns, mid, B3_COL_WHITE, alpha=B3_FILL_ALPHA)); mid += 1
                    continue

                # (C) 그 외 B3 폴리곤: 얇은 외곽선 + 반투명 채움(흰색)
                outline = Marker(type=Marker.LINE_LIST)
                outline.scale.x = B3_OUTLINE_W

                fill = Marker(type=Marker.TRIANGLE_LIST)
                fill.scale.x = fill.scale.y = fill.scale.z = 1.0

                for p in polys:
                    p2 = _shift_polygon_keep_holes(p, xo, yo)
                    coords = list(p2.exterior.coords)
                    for a, b in zip(coords[:-1], coords[1:]):
                        outline.points.append(Point(x=float(a[0]), y=float(a[1]), z=0.02))
                        outline.points.append(Point(x=float(b[0]), y=float(b[1]), z=0.02))
                    fill.points.extend(_triangulate_to_points(p2, z=0.01))

                ma.markers.append(self._finish(outline, ns, mid, B3_COL_WHITE, alpha=1.0)); mid += 1
                ma.markers.append(self._finish(fill,    ns, mid, B3_COL_WHITE, alpha=0.35)); mid += 1
                continue

            # ===================== A5 (PARKINGLOT) =====================
            if is_a5:
                if gt not in ("Polygon", "MultiPolygon"):
                    continue

                polys = [geom] if gt == "Polygon" else list(geom.geoms)

                outline = Marker(type=Marker.LINE_LIST)
                outline.scale.x = A5_OUTLINE_W

                fill = None
                if A5_DRAW_FILL:
                    fill = Marker(type=Marker.TRIANGLE_LIST)
                    fill.scale.x = fill.scale.y = fill.scale.z = 1.0

                for p in polys:
                    p2 = _shift_polygon_keep_holes(p, xo, yo)

                    rings = [p2.exterior] + list(p2.interiors)
                    for ring in rings:
                        coords = list(ring.coords)
                        if len(coords) < 2:
                            continue
                        for a, b in zip(coords[:-1], coords[1:]):
                            outline.points.append(Point(x=float(a[0]), y=float(a[1]), z=A5_Z_LINE))
                            outline.points.append(Point(x=float(b[0]), y=float(b[1]), z=A5_Z_LINE))

                    if A5_SYNTH_SLOTS_ENABLE:
                        slots = self._a5_synthesize_slots(p2, n=A5_SLOT_N)
                        for sp in slots:
                            coords = list(sp.exterior.coords)
                            for a, b in zip(coords[:-1], coords[1:]):
                                outline.points.append(Point(x=float(a[0]), y=float(a[1]), z=A5_Z_LINE))
                                outline.points.append(Point(x=float(b[0]), y=float(b[1]), z=A5_Z_LINE))

                    if fill is not None:
                        fill.points.extend(_triangulate_to_points(p2, z=A5_Z_FILL))

                ma.markers.append(self._finish(outline, ns, mid, A5_COL_WHITE, alpha=1.0)); mid += 1
                if fill is not None and len(fill.points) >= 3:
                    ma.markers.append(self._finish(fill, ns, mid, A5_FILL_COL, alpha=A5_FILL_ALPHA)); mid += 1
                continue

            # ===================== 기존(B2 포함) =====================
            if gt in ("LineString", "LinearRing", "MultiLineString"):
                lines = [geom] if gt != "MultiLineString" else list(geom.geoms)

                if is_b2 and b2_type_col is not None:
                    rgba, width_mul, pattern, _ = _parse_b2_type(row.get(b2_type_col))
                    dashed = _is_dashed_pattern(pattern)
                    if pattern == 2:
                        dash_len, gap_len, sample_step = 1.8, 1.2, 0.6
                    elif pattern in (3, 4):
                        dash_len, gap_len, sample_step = 1.1, 0.9, 0.5
                    else:
                        dash_len, gap_len, sample_step = 2.0, 1.5, 0.5

                    for ln_geom in lines:
                        coords = [(p[0]-xo, p[1]-yo) for p in ln_geom.coords]
                        line = SLineString(coords)
                        if not dashed:
                            m = Marker(type=Marker.LINE_STRIP)
                            m.scale.x = LINE_W_B2 * width_mul
                            m.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in line.coords]
                            ma.markers.append(self._finish(m, ns, mid, rgba)); mid += 1
                        else:
                            m = Marker(type=Marker.LINE_LIST)
                            m.scale.x = LINE_W_B2 * width_mul
                            m.points = _make_dashed_line_list_points(
                                line, dash_len=dash_len, gap_len=gap_len, sample_step=sample_step, z=0.02
                            )
                            ma.markers.append(self._finish(m, ns, mid, rgba)); mid += 1
                    continue

                for ln_geom in lines:
                    coords = [(p[0]-xo, p[1]-yo) for p in ln_geom.coords]
                    m = Marker(type=Marker.LINE_STRIP); m.scale.x = LINE_W_DEFAULT
                    m.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in coords]
                    ma.markers.append(self._finish(m, ns, mid, col)); mid += 1
                continue

            if gt in ("Polygon", "MultiPolygon"):
                polys = [geom] if isinstance(geom, Polygon) else geom.geoms
                for poly in polys:
                    ln = Marker(type=Marker.LINE_STRIP); ln.scale.x = LINE_W_POLY
                    ln.points = [Point(x=p[0]-xo, y=p[1]-yo, z=0.01) for p in poly.exterior.coords]
                    ma.markers.append(self._finish(ln, ns, mid, col)); mid += 1

                fill = Marker(type=Marker.TRIANGLE_LIST)
                fill.scale.x = fill.scale.y = fill.scale.z = 1.0
                for poly in polys:
                    for tri in triangulate(poly):
                        for xyz in tri.exterior.coords[:3]:
                            x, y = xyz[:2]
                            fill.points.append(Point(x=x-xo, y=y-yo, z=0.0))
                ma.markers.append(self._finish(fill, ns, mid, col, 0.8)); mid += 1
                continue

            if gt == "Point":
                s = Marker(type=Marker.SPHERE)
                s.scale.x = s.scale.y = s.scale.z = POINT_R
                s.pose.position.x = geom.x - xo
                s.pose.position.y = geom.y - yo
                s.pose.position.z = 0.1
                ma.markers.append(self._finish(s, ns, mid, col)); mid += 1

        return ma

    @staticmethod
    def _finish(m: Marker, ns: str, mid: int, rgba: ColorRGBA, alpha: float | None = None) -> Marker:
        m.header.frame_id = FRAME_ID
        m.ns, m.id = ns, mid
        if (m.pose.orientation.x == 0.0 and m.pose.orientation.y == 0.0 and
            m.pose.orientation.z == 0.0 and m.pose.orientation.w == 0.0):
            m.pose.orientation.w = 1.0
        m.lifetime.sec = 0
        a = (alpha if alpha is not None else rgba.a)
        m.color = ColorRGBA(r=rgba.r, g=rgba.g, b=rgba.b, a=a)

        # TRIANGLE_LIST는 per-vertex colors를 채워 검게 보이는 현상 회피
        if m.type == Marker.TRIANGLE_LIST:
            if len(m.points) > 0:
                c = ColorRGBA(r=rgba.r, g=rgba.g, b=rgba.b, a=a)
                m.colors = [c] * len(m.points)
        return m

    def _tick(self):
        for topic, pub in self._pubs.items():
            pub.publish(self._cache[topic])

def main(argv: List[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = HDMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
