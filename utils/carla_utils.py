import atexit
import collections
import json
import math
import os
import queue
import re
import threading
import time
import traceback

import carla
import numpy as np

# ======================================================================
#   论文插图采集开关 (FIG_CAPTURE=1 专用)
#   ------------------------------------------------------------------
#   未设置时: 本模块所有原有函数行为逐字节不变 (draw_planned_trj 仍不画线),
#             FigCaptureSession 不会被任何入口构造。
#   设置 FIG_CAPTURE=1 时: draw_planned_trj 真正调用 world.debug.draw_line,
#             env.reset() 构造 FigCaptureSession (BEV 相机 + sidecar 缓冲)。
#   FIG_LINE_SCALE: 世界系画线粗细缩放 (默认 1.0 = 规格值 0.08 / 0.06 m)。
# ======================================================================
FIG_CAPTURE = os.environ.get('FIG_CAPTURE', '0') == '1'
_FIG_LINE_SCALE = float(os.environ.get('FIG_LINE_SCALE', '1.0') or 1.0)
# 世界系画图规格 (2026-09-07 协调人反馈: 45 m 高度下 0.08 m 线仅 ~1 px, 且被车顶遮挡)
FIG_REF_THICK = 0.30       # 参考线 / MPC 预测线粗细 [m]
FIG_BOX_THICK = 0.15       # ego / 周车框线粗细 [m]
FIG_LINE_Z_LIFT = 0.3      # 折线抬到车顶之上的高度 [m]
FIG_PT_SIZE = float(os.environ.get('FIG_PT_SIZE', '0.20') or 0.20)   # IGPR 预测点尺寸
# 调用点历史颜色 -> 饱和色 (参考线红, MPC 预测青)
_FIG_LINE_COLORS = {(200, 0, 0): (255, 0, 0), (0, 233, 222): (0, 255, 255)}


class FigOverviewDone(Exception):
    """FIG_OVERVIEW_ONLY=1: 全景帧已落盘, 主动结束本集 (由 udmc_main 主循环 except 收尾)。"""

def draw_planned_trj(world, x_trj, car_z, color=(200,0,0)):
    color = carla.Color(r=color[0],g=color[1],b=color[2],a=0)
    length = x_trj.shape[0]
    xx = x_trj[:,0]
    yy = -x_trj[:,1]
    if FIG_CAPTURE:
        # 论文插图采集: 参考线 -> (255,0,0) 隔点画且只画前 30 m; MPC 预测 -> (0,255,255) 逐点画。
        # 画在车顶之上 0.3 m; draw_line 为异步 RPC, 每步总计 ≈ 15 次。
        r, g, b = _FIG_LINE_COLORS.get((color.r, color.g, color.b), (color.r, color.g, color.b))
        stride = 2 if (r, g, b) == (255, 0, 0) else 1
        z_fig = float(car_z + 1.0 + FIG_LINE_Z_LIFT)
        col = carla.Color(r=r, g=g, b=b, a=255)
        acc = 0.0
        for i in range(stride, length, stride):
            acc += math.hypot(float(xx[i] - xx[i - stride]), float(yy[i] - yy[i - stride]))
            if acc > 30.0:
                break
            world.debug.draw_line(begin=carla.Location(float(xx[i - stride]), float(yy[i - stride]), z_fig),
                                  end=carla.Location(float(xx[i]), float(yy[i]), z_fig),
                                  thickness=FIG_REF_THICK * _FIG_LINE_SCALE, color=col, life_time=0.1)
        return
    for i in range(1, length):
        begin = carla.Location(float(xx[i-1]), float(yy[i-1]), float(car_z+1))
        end = carla.Location(float(xx[i]), float(yy[i]), float(car_z+1))
        # world.debug.draw_line(begin=begin, end=end, thickness=0.08, color=color, life_time=0.1)

def carla_vector_to_rh_vector(position, yaw, velocity=None):
    """
    Convert a carla location to a right-hand system
    position: x, y, z
    yaw: yaw(degree)
    velocity: vx, vy, omega
    """
    x = position[0]
    y = -position[1]
    yaw = -np.radians(yaw)

    if velocity is not None:
        vx = velocity[0]
        vy = -velocity[1]
        omega = -velocity[2]
        
        return [x, y, yaw, vx, vy, omega]
    
    return [x, y, yaw, 0, 0, 0]

def spawn_vehicles_around_ego_vehicles(ego_vehicle_transform, max_dis, min_dis, spawn_points, numbers_of_vehicles):
    """
    parameters:
    ego_vehicle :: your target vehicle
    max_dis :: the distance max limitation between ego-vehicle and other free-vehicles
    min_dis :: the distance min limitation
    spawn_points :: the available spawn points in current map
    numbers_of_vehicles :: the number of free-vehicles around ego-vehicle that you need
    """
    np.random.shuffle(spawn_points) # shuffle all the spawn points
    ego_location = ego_vehicle_transform.location
    accessible_points = []
    for spawn_point in spawn_points:
        dis = math.sqrt((ego_location.x-spawn_point.location.x)**2 + (ego_location.y-spawn_point.location.y)**2)
        # it also can include z-coordinate,but it is unnecessary
        if dis < max_dis and dis > min_dis and (spawn_point not in accessible_points):
            accessible_points.append(spawn_point)

    transform_list = [] # keep the spawned vehicle in vehicle_list, because we need to link them with traffic_manager
    if len(accessible_points) < numbers_of_vehicles:
        # if your radius is relatively small,the satisfied points may be insufficient
        numbers_of_vehicles = len(accessible_points)

    for i in range(numbers_of_vehicles): # generate the free vehicle
        point = accessible_points[i]
        transform_list.append(point)

    print("spawned vehicles number: ", len(transform_list))
    return transform_list


# ======================================================================
#   世界系画图 helper (FIG_CAPTURE=1 专用; 输入均为 CARLA 世界系坐标)
# ======================================================================
def _rect_vertices_carla(cx, cy, yaw_rad, L, W):
    """中心 + 航向(rad, CARLA 系) + 长宽 -> 4 顶点 (CARLA 世界系, 顺序闭合)。"""
    hx, hy = L / 2.0, W / 2.0
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    local = [(+hx, +hy), (-hx, +hy), (-hx, -hy), (+hx, -hy)]
    return [(cx + c * lx - s * ly, cy + s * lx + c * ly) for lx, ly in local]


def draw_polygon(world, vertices_xy_carla, z, color=(0, 200, 0), life_time=0.1, thickness=0.06):
    """闭合多边形 (CARLA 世界系顶点列表 [(x, y), ...], 高度 z)。"""
    pts = [(float(v[0]), float(v[1])) for v in vertices_xy_carla]
    n = len(pts)
    if n < 2:
        return
    col = carla.Color(r=int(color[0]), g=int(color[1]), b=int(color[2]), a=255)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        world.debug.draw_line(carla.Location(x=x0, y=y0, z=float(z)),
                              carla.Location(x=x1, y=y1, z=float(z)),
                              thickness=float(thickness) * _FIG_LINE_SCALE,
                              color=col, life_time=float(life_time))


def draw_polyline_carla(world, xy_carla, z, color=(200, 0, 0), life_time=0.1, thickness=0.08):
    """开放折线 (CARLA 世界系 (N,2) 或 (N,3): 第三列为逐点高度)。"""
    arr = np.asarray(xy_carla, dtype=float)
    if arr.ndim != 2 or arr.shape[0] < 2:
        return
    col = carla.Color(r=int(color[0]), g=int(color[1]), b=int(color[2]), a=255)
    if arr.shape[1] >= 3:
        zs = arr[:, 2] + float(z)                 # 第三列为逐点高度, z 为附加偏移
    else:
        zs = np.full(arr.shape[0], float(z))      # 两列输入: z 为绝对高度
    for i in range(1, arr.shape[0]):
        world.debug.draw_line(carla.Location(x=float(arr[i - 1, 0]), y=float(arr[i - 1, 1]), z=float(zs[i - 1])),
                              carla.Location(x=float(arr[i, 0]), y=float(arr[i, 1]), z=float(zs[i])),
                              thickness=float(thickness) * _FIG_LINE_SCALE,
                              color=col, life_time=float(life_time))


# ======================================================================
#   FigCaptureSession —— 论文插图采集会话 (FIG_CAPTURE=1 专用)
#   ------------------------------------------------------------------
#   生命周期: env.reset() 构造 -> env.step() 每步 before_tick/after_tick
#             -> agent 每步 stash_obs/capture_step -> env.clean_all_actors()
#             或进程退出 (atexit) 时 finalize (幂等: 写 sidecar + 销毁相机)。
#   相机: sensor.camera.rgb 1024x1024 fov90, 不挂载, 每步移到 ego 正上方
#         (pitch -90, yaw 0, roll 0 固定朝向), 队列取当拍帧, 每 FIG_EVERY 步存 PNG;
#         第 20 步另起一台全景相机于路线包围盒中心 (z=FIG_OVERVIEW_Z) 抓一帧。
#   sidecar (<LOG_OUTPUT 去 .txt>_fig.npz) 键名/形状为跨模块契约, 见 finalize。
#   坐标: 对外一律 CARLA 世界系 (x, y, yaw_deg); 右手系输入在本类内转回。
# ======================================================================
class FigCaptureSession:
    N_OBS = 6            # sidecar 记录的最近车辆数
    N_PRED = 10          # IGPR 预测步数
    N_MPC = 11           # MPC 预测点数 (horizon 0..10)
    REF_EVERY = 20       # 混合参考线抽样间隔 (行)
    OVERVIEW_STEP = 20   # 全景抓拍步
    ID_REFRESH = 20      # 车辆 id 列表刷新间隔 (步, ≈1 s)

    def __init__(self, world, ego_vehicle, dt, tag=None, log_output=None):
        import cv2  # 延迟 import: 仅 FIG_CAPTURE=1 时需要
        self._cv2 = cv2
        self._world = world
        self._ego = ego_vehicle
        self._dt = float(dt)
        env = os.environ
        self._tag = tag or env.get('FIG_TAG') or ('fig_' + time.strftime('%Y%m%d_%H%M%S'))
        self._cam_z = float(env.get('FIG_CAM_Z', '45') or 45.0)
        self._every = max(int(float(env.get('FIG_EVERY', '10') or 10)), 1)
        _ov_env = (env.get('FIG_OVERVIEW_Z') or '').strip()
        self._ov_z_fixed = float(_ov_env) if _ov_env else None   # 显式给定则不自适应
        self._ov_z = self._ov_z_fixed if self._ov_z_fixed is not None else 150.0
        self._ov_only = env.get('FIG_OVERVIEW_ONLY', '0') == '1'
        self._ov_layer_life = 0.3      # 全景抓拍图层寿命 [s]: 只在抓拍窗口内存在
        self._fov = 90.0
        self._res = 1024
        frame_root = env.get('FIG_FRAME_ROOT') or os.path.join('images', 'fig_capture')
        self._frame_dir = os.path.abspath(os.path.join(frame_root, self._tag))
        os.makedirs(self._frame_dir, exist_ok=True)
        log_output = log_output if log_output is not None else env.get('LOG_OUTPUT', '')
        if log_output:
            base = log_output[:-4] if log_output.endswith('.txt') else log_output
            self._npz_path = os.path.abspath(base + '_fig.npz')
        else:
            self._npz_path = os.path.abspath(os.path.join('logs', self._tag + '_fig.npz'))

        # ---- meta: 优先环境变量, 其次从 tag "<METHOD>_<scenario>_s<seed>" 解析 ----
        m = re.match(r'^([A-Za-z0-9\-]+)_(.+)_s(\d+)$', self._tag)
        scenario = env.get('FIG_SCENARIO') or (m.group(2) if m else '')
        method = env.get('FIG_METHOD') or (m.group(1) if m else '')
        seed_s = env.get('RANDOM_SEED') or (m.group(3) if m else '')
        try:
            seed = int(seed_s)
        except (TypeError, ValueError):
            seed = None
        self._meta = dict(
            scenario=scenario, seed=seed, method=method, dt=self._dt,
            frame_dir=self._frame_dir, cam_z=self._cam_z, fov=self._fov,
            resolution=[self._res, self._res], tag=self._tag,
            every=self._every, overview_z=self._ov_z, overview_step=self.OVERVIEW_STEP,
            overview_only=self._ov_only, line_scale=_FIG_LINE_SCALE,
            ref_thick=FIG_REF_THICK, box_thick=FIG_BOX_THICK, pt_size=FIG_PT_SIZE,
            bev_axes='camera at (ego.x, ego.y, cam_z) pitch=-90 yaw=0 roll=0; '
                     'image up = +x_carla, image right = +y_carla; '
                     'ground half-extent [m] = cam_z*tan(fov/2)',
            frame_row_semantics='bev_<k> is rendered by the world tick that follows '
                                'log row k (0-based); drawings show row k plan, actor '
                                'poses are one dt later than row k',
            frame='CARLA world frame: x, y [m], yaw_deg; y_carla = -y_rh',
        )

        # ---- sidecar 缓冲 ----
        self._t, self._ego_rows, self._obs, self._obs_id = [], [], [], []
        self._pred, self._pred_std, self._mpc, self._min_dist = [], [], [], []
        self._ref_raw0 = None
        self._ref_blend0 = None
        self._ref_t, self._ref_xy = [], []
        self._obs_stash = ([], [], [])
        self._log_ref = None

        # ---- 路线 / 车辆缓存 / 计时 ----
        self._route = None              # (N,3) CARLA x, y, z
        self._route_center = None
        self._veh_ids = []              # 非 ego 车辆 id (每 ID_REFRESH 步刷新一次 get_actors)
        self._veh_bb = {}               # id -> (off_x, off_y, off_z, ext_x, ext_y, ext_z)
        self._veh_refresh_row = -10 ** 9
        self._timers = collections.defaultdict(float)
        self._timer_n = collections.defaultdict(int)

        # ---- 相机: 帧登记表 (frame_id -> 路径, 回调线程消费) + 写盘线程 ----
        self._want = {}
        self._want_lock = threading.Lock()
        self._want_last = None          # 最近登记的 frame id (tick 后与快照 frame 比对, 自检)
        self._frame_mismatch = 0
        self._writer_q = queue.Queue()
        self._writer_qmax = 0
        self._n_frames = 0
        self._writer = threading.Thread(target=self._writer_loop, name='fig_writer', daemon=True)
        self._writer.start()
        self._bev_cam = None
        self._ov_cam, self._ov_saved, self._ov_done, self._ov_spawn_step = None, False, False, None
        self._step = 0
        self._finalized = False

        self._bev_cam = self._make_cam(self._bev_transform())
        self._bev_cam.listen(self._bev_cb)
        atexit.register(self.finalize)
        print('[FigCapture] session tag=%s frame_dir=%s npz=%s cam_z=%.1f every=%d'
              % (self._tag, self._frame_dir, self._npz_path, self._cam_z, self._every))

    # ------------------------------------------------------------------
    #   相机 / 帧管线 (主线程零阻塞: 帧由回调线程按登记表交给写盘线程)
    # ------------------------------------------------------------------
    def _make_cam(self, transform):
        bp = self._world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(self._res))
        bp.set_attribute('image_size_y', str(self._res))
        bp.set_attribute('fov', str(self._fov))
        # 关闭泛光/镜头光晕/运动模糊: 饱和色 debug 线在默认后处理下会晕成光团盖住 ego
        for attr, val in (('bloom_intensity', '0.0'), ('lens_flare_intensity', '0.0'),
                          ('motion_blur_intensity', '0.0')):
            if bp.has_attribute(attr):
                bp.set_attribute(attr, val)
        return self._world.spawn_actor(bp, transform)   # 不挂载, 姿态固定

    @staticmethod
    def _to_bgr(image):
        return np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
            (image.height, image.width, 4))[:, :, :3].copy()   # BGRA -> BGR

    def _bev_cb(self, image):
        """BEV 相机回调 (传感器线程): 只处理登记过的 frame, 其余直接丢弃 (零拷贝)。"""
        with self._want_lock:
            path = self._want.pop(int(image.frame), None)
        if path is not None:
            self._writer_q.put((path, self._to_bgr(image)))

    def _ov_cb(self, image):
        """全景相机回调: 取第一帧即存。"""
        if self._ov_saved:
            return
        self._ov_saved = True
        self._meta['overview_frame'] = int(image.frame)
        self._writer_q.put((os.path.join(self._frame_dir, 'overview.png'), self._to_bgr(image)))

    def _writer_loop(self):
        while True:
            item = self._writer_q.get()
            if item is None:
                return
            path, arr = item
            try:
                self._cv2.imwrite(path, arr)
                self._n_frames += 1
            except Exception as e:
                print('[FigCapture] imwrite failed:', path, e)

    @staticmethod
    def _destroy_cam(cam):
        if cam is None:
            return
        try:
            cam.stop()
        except Exception:
            pass
        try:
            cam.destroy()
        except Exception:
            pass

    def _bev_transform(self):
        loc = self._ego.get_location()
        return carla.Transform(carla.Location(x=loc.x, y=loc.y, z=self._cam_z),
                               carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0))

    def before_tick(self):
        """env.step(): 施加控制前调用 —— 移 BEV 相机 / 登记本 tick 要存的帧 / 全景相机。"""
        if self._finalized:
            return
        t0 = time.perf_counter()
        k = self._step
        try:
            if self._bev_cam is not None:
                self._bev_cam.set_transform(self._bev_transform())      # 异步 RPC
            if k % self._every == 0:
                fid = int(self._world.get_snapshot().frame) + 1       # 本 tick 的 frame (客户端缓存态)
                path = os.path.join(self._frame_dir, 'bev_%06d_t%.2f.png' % (k, (k + 1) * self._dt))
                self._want_last = fid
                with self._want_lock:
                    self._want[fid] = path
                    for old in [f for f in self._want if f < fid - 50]:   # 漏帧登记清理
                        self._want.pop(old, None)
            if not self._ov_done:
                if self._ov_cam is None and k >= self.OVERVIEW_STEP - 2:
                    if self._route_center is not None:
                        cx, cy = self._route_center
                    else:
                        loc = self._ego.get_location()
                        cx, cy = loc.x, loc.y
                    self._meta['overview_center'] = [float(cx), float(cy)]
                    self._meta['overview_z'] = float(self._ov_z)
                    self._ov_cam = self._make_cam(carla.Transform(
                        carla.Location(x=float(cx), y=float(cy), z=self._ov_z),
                        carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)))
                    self._ov_cam.listen(self._ov_cb)
                    self._ov_spawn_step = k
                if self._ov_cam is not None:
                    if self._ov_saved:
                        self._destroy_cam(self._ov_cam)
                        self._ov_cam = None
                        self._ov_done = True
                        self._meta['overview_step_actual'] = int(k)
                    elif k >= self._ov_spawn_step + 40:
                        print('[FigCapture] overview frame never arrived, give up')
                        self._destroy_cam(self._ov_cam)
                        self._ov_cam = None
                        self._ov_done = True
                    elif self._route is not None:
                        self._draw_overview_layer()      # 抓拍窗口内 (通常 1-2 步) 重画
        except Exception as e:
            print('[FigCapture] before_tick failed:', e)
        self._timers['before_tick'] += time.perf_counter() - t0
        self._timer_n['before_tick'] += 1

    def after_tick(self, sim_time):
        """env.step(): world.tick() 之后调用 —— 不等待任何帧; 仅处理仅全景模式的收尾。"""
        if self._finalized:
            return
        k = self._step
        self._step += 1
        if k % self._every == 0 and self._want_last is not None:
            # 自检: tick 后快照 frame 应等于登记的 frame (客户端缓存态读取, 无 RPC)
            try:
                if int(self._world.get_snapshot().frame) != self._want_last:
                    self._frame_mismatch += 1
            except Exception:
                pass
        if self._ov_only and self._ov_done and k >= self.OVERVIEW_STEP + 5:
            raise FigOverviewDone('[FigCapture] FIG_OVERVIEW_ONLY: overview captured, stop episode at step %d' % k)

    # ------------------------------------------------------------------
    #   路线 / 画图
    # ------------------------------------------------------------------
    def set_route(self, route_xyz_carla):
        """agent.plan_route(): 全局路线 (N,3) CARLA x,y,z; 只记录包围盒 (全景相机高度/中心)。"""
        arr = np.asarray(route_xyz_carla, dtype=float).reshape(-1, 3)
        if arr.shape[0] < 2:
            return
        self._route = arr
        lo, hi = arr[:, :2].min(axis=0), arr[:, :2].max(axis=0)
        self._route_center = (float((lo[0] + hi[0]) / 2.0), float((lo[1] + hi[1]) / 2.0))
        self._meta['route_bbox'] = [float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1])]
        self._meta['route_n'] = int(arr.shape[0])
        # 全景相机高度自适应: z = clamp(0.75 * max(bbox_w, bbox_h) + 25, 60, 170)
        if self._ov_z_fixed is None:
            span = float(max(hi[0] - lo[0], hi[1] - lo[1]))
            self._ov_z = float(min(max(0.75 * span + 25.0, 60.0), 170.0))
        self._meta['overview_z'] = self._ov_z
        self._meta['overview_center'] = [self._route_center[0], self._route_center[1]]

    def _draw_overview_layer(self):
        """全景抓拍专用图层: 粗橙色全程路线 (0.006*z m) + 起终点 1.5 m 实心方块 (绿/红)。
        寿命 0.3 s, 仅在全景相机存活的 1-2 步内重画; 常规集中只会出现在 bev_000020 一帧。"""
        arr = self._route
        life = self._ov_layer_life
        thick = 0.006 * float(self._ov_z)
        draw_polyline_carla(self._world, arr, 0.5, color=(255, 140, 0), life_time=life, thickness=thick)
        for (x, y, zz), col in ((arr[0], (0, 200, 0)), (arr[-1], (220, 0, 0))):
            bb = carla.BoundingBox(carla.Location(x=float(x), y=float(y), z=float(zz + 0.8)),
                                   carla.Vector3D(0.75, 0.75, 0.3))
            self._world.debug.draw_box(bb, carla.Rotation(), thickness=max(thick, 0.5),
                                       color=carla.Color(r=col[0], g=col[1], b=col[2], a=255),
                                       life_time=life)

    def draw_polyline_rh(self, xy_rh, z, color=(255, 0, 0), thickness=FIG_REF_THICK, stride=1, max_len=None):
        """右手系折线 (N,2) -> CARLA 系画线 (UDMC 臂参考线 / MPC 预测线用)。stride 隔点, max_len 长度上限 [m]。"""
        arr = np.asarray(xy_rh, dtype=float)
        if arr.ndim != 2 or arr.shape[0] < 2:
            return
        arr = arr[::max(int(stride), 1)]
        if max_len is not None and arr.shape[0] > 1:
            seg = np.hypot(np.diff(arr[:, 0]), np.diff(arr[:, 1]))
            keep = int(np.searchsorted(np.cumsum(seg), float(max_len))) + 1
            arr = arr[:max(keep + 1, 2)]
        color = _FIG_LINE_COLORS.get(tuple(int(c) for c in color), color)
        draw_polyline_carla(self._world, np.column_stack([arr[:, 0], -arr[:, 1]]), z,
                            color=color, life_time=0.1, thickness=thickness)

    def draw_box_carla(self, cx, cy, yaw_deg, L, W, z, color, thickness=FIG_BOX_THICK):
        draw_polygon(self._world, _rect_vertices_carla(cx, cy, math.radians(yaw_deg), L, W), z,
                     color=color, life_time=0.1, thickness=thickness)

    # ------------------------------------------------------------------
    #   数据 (agent 调用)
    # ------------------------------------------------------------------
    def bind_log(self, log_list):
        """绑定 agent._log_data (同一 list 对象), finalize 时取第 1/16 列。"""
        self._log_ref = log_list

    def stash_obs(self, obs_rh, obs_infer_rh, obs_stds=None):
        """env.get_obs() 之后立即调用 (过滤前的全集, 右手系)。"""
        self._obs_stash = (list(obs_rh) if obs_rh is not None else [],
                           list(obs_infer_rh) if obs_infer_rh is not None else [],
                           list(obs_stds) if obs_stds is not None else [])

    def _refresh_vehicle_ids(self, row):
        """每 ID_REFRESH 步做一次 get_actors (唯一的 actor 列表 RPC), 缓存 id 与包围盒。"""
        acts = self._world.get_actors().filter('vehicle.*')
        ids = []
        for a in acts:
            if a.id == self._ego.id:
                continue
            ids.append(int(a.id))
            if a.id not in self._veh_bb:
                bb = a.bounding_box
                self._veh_bb[int(a.id)] = (bb.location.x, bb.location.y, bb.location.z,
                                           bb.extent.x, bb.extent.y, bb.extent.z)
        self._veh_ids = ids
        self._veh_refresh_row = row

    def capture_step(self, t, mpc_states_rh, ref_raw_rh, ref_blend_rh):
        """agent.run_step() 末尾 (log 行 append 之后) 调用: 组装 sidecar 行 + 画框。
        每步只读一次 world.get_snapshot() (客户端缓存态), 周车姿态取 ActorSnapshot;
        任何异常都保证追加一行 (NaN 填充), 保持与主日志逐行对齐。"""
        n_obs, n_pred, n_mpc = self.N_OBS, self.N_PRED, self.N_MPC
        ego4 = [np.nan] * 4
        obs_arr = np.full((n_obs, 4), np.nan)
        id_arr = np.full((n_obs,), -1, dtype=np.int64)
        pred_arr = np.full((n_obs, n_pred, 3), np.nan)
        std_arr = np.full((n_obs,), np.nan)
        mpc_arr = np.full((n_mpc, 2), np.nan)
        min_dist = 999.0
        t0 = time.perf_counter()
        try:
            row = len(self._t)
            snap = self._world.get_snapshot()
            if row - self._veh_refresh_row >= self.ID_REFRESH or not self._veh_ids:
                self._refresh_vehicle_ids(row)
            es = snap.find(self._ego.id)
            ego_tf = es.get_transform() if es is not None else self._ego.get_transform()
            ego_vel = es.get_velocity() if es is not None else self._ego.get_velocity()
            ego_loc = ego_tf.location
            ego4 = [ego_loc.x, ego_loc.y, ego_tf.rotation.yaw, math.hypot(ego_vel.x, ego_vel.y)]
            t1 = time.perf_counter()

            # ---- 最近 N_OBS 辆车: 快照位置一次性取出, numpy 排序 (3D 距离, 与日志同口径) ----
            snaps, xyz = [], []
            for aid in self._veh_ids:
                s = snap.find(aid)
                if s is None:
                    continue
                l = s.get_transform().location
                snaps.append((aid, s))
                xyz.append((l.x, l.y, l.z))
            near = []
            if snaps:
                xyz = np.asarray(xyz, dtype=float)
                d = np.sqrt(((xyz - np.array([ego_loc.x, ego_loc.y, ego_loc.z])) ** 2).sum(axis=1))
                order = np.argsort(d)[:n_obs]
                min_dist = float(d[order[0]])
                near = [(snaps[j][0], snaps[j][1]) for j in order]

            obs_raw, infer_raw, std_raw = self._obs_stash
            boxes = []
            for i, (aid, s) in enumerate(near):
                tf = s.get_transform()
                v = s.get_velocity()
                obs_arr[i] = [tf.location.x, tf.location.y, tf.rotation.yaw, math.hypot(v.x, v.y)]
                id_arr[i] = aid
                # 位置匹配 get_obs 的右手系条目 (容差 1 m^2), 取其 IGPR 预测
                ax, ay = tf.location.x, -tf.location.y
                best_j, best_d2 = -1, 1.0
                for j, o in enumerate(obs_raw):
                    d2 = (float(o[0]) - ax) ** 2 + (float(o[1]) - ay) ** 2
                    if d2 < best_d2:
                        best_d2, best_j = d2, j
                if 0 <= best_j < len(infer_raw):
                    p = np.asarray(infer_raw[best_j], dtype=float).reshape(-1, 3)[:n_pred]
                    pred_arr[i, :p.shape[0], 0] = p[:, 0]
                    pred_arr[i, :p.shape[0], 1] = -p[:, 1]
                    pred_arr[i, :p.shape[0], 2] = -np.degrees(p[:, 2])
                    if best_j < len(std_raw):
                        std_arr[i] = float(std_raw[best_j])
                bb = self._veh_bb.get(aid)
                if bb is not None:
                    c = tf.transform(carla.Location(x=bb[0], y=bb[1], z=bb[2]))
                    boxes.append((c.x, c.y, tf.rotation.yaw, 2.0 * bb[3], 2.0 * bb[4], c.z + bb[5] + 0.5))
            t2 = time.perf_counter()

            # ---- MPC 预测 (右手系 (11,5) -> CARLA x,y) ----
            if mpc_states_rh is not None:
                xm = np.asarray(mpc_states_rh, dtype=float)
                if xm.ndim == 2:
                    if xm.shape[0] != n_mpc and xm.shape[1] == n_mpc:
                        xm = xm.T
                    mm = min(xm.shape[0], n_mpc)
                    mpc_arr[:mm, 0] = xm[:mm, 0]
                    mpc_arr[:mm, 1] = -xm[:mm, 1]

            # ---- 参考线: 首步存 raw/blend, 每 REF_EVERY 行存一次 blend ----
            if ref_blend_rh is not None:
                blend = np.asarray(ref_blend_rh, dtype=float).reshape(-1, 2)
                blend_c = np.column_stack([blend[:, 0], -blend[:, 1]])
                if row == 0:
                    self._ref_blend0 = blend_c
                    raw = np.asarray(ref_raw_rh if ref_raw_rh is not None else ref_blend_rh, dtype=float).reshape(-1, 2)
                    self._ref_raw0 = np.column_stack([raw[:, 0], -raw[:, 1]])
                if row % self.REF_EVERY == 0:
                    self._ref_t.append(float(t))
                    self._ref_xy.append(blend_c)
            t3 = time.perf_counter()

            # ---- 世界系画框 (异步 RPC): ego OBCA 框 (4.5x2.0, 绿) + 最近 N_OBS 车 bbox (黄) ----
            self.draw_box_carla(ego_loc.x, ego_loc.y, ego_tf.rotation.yaw, 4.5, 2.0,
                                ego_loc.z + 2.0, color=(0, 200, 0))
            for (cx, cy, yaw, L, W, z) in boxes:
                self.draw_box_carla(cx, cy, yaw, L, W, z, color=(240, 200, 0))
            t4 = time.perf_counter()
            self._timers['snapshot+ids'] += t1 - t0
            self._timers['nearest+pred'] += t2 - t1
            self._timers['mpc+refs'] += t3 - t2
            self._timers['draw_boxes'] += t4 - t3
        except Exception as e:
            print('[FigCapture] capture_step failed (row NaN-filled):', e)
        finally:
            self._timers['capture_step'] += time.perf_counter() - t0
            self._timer_n['capture_step'] += 1
            self._t.append(float(t))
            self._ego_rows.append(ego4)
            self._obs.append(obs_arr)
            self._obs_id.append(id_arr)
            self._pred.append(pred_arr)
            self._pred_std.append(std_arr)
            self._mpc.append(mpc_arr)
            self._min_dist.append(float(min_dist))

    # ------------------------------------------------------------------
    #   收尾 (幂等)
    # ------------------------------------------------------------------
    def finalize(self):
        if self._finalized:
            return
        self._finalized = True
        for cam in (self._bev_cam, self._ov_cam):
            self._destroy_cam(cam)
        self._bev_cam, self._ov_cam = None, None
        try:
            self._writer_q.put(None)
            self._writer.join(timeout=60.0)
        except Exception:
            pass
        try:
            self._write_npz()
        except Exception as e:
            print('[FigCapture] sidecar write failed:', e)
            traceback.print_exc()
        self._print_timing()

    def _print_timing(self):
        n = max(self._timer_n.get('capture_step', 0), 1)
        parts = ', '.join('%s %.2f' % (k, 1000.0 * v / n) for k, v in self._timers.items() if k != 'capture_step' and k != 'before_tick')
        print('[FigCapture] timing ms/step over %d steps: capture_step %.2f (%s) | before_tick %.2f | frames written %d | frame-id mismatches %d'
              % (n, 1000.0 * self._timers.get('capture_step', 0.0) / n, parts,
                 1000.0 * self._timers.get('before_tick', 0.0) / max(self._timer_n.get('before_tick', 0), 1),
                 self._n_frames, self._frame_mismatch))

    def _write_npz(self):
        T = len(self._t)
        t = np.asarray(self._t, dtype=float)
        collision = np.zeros((T,), dtype=np.int64)
        log = None
        if self._log_ref is not None and len(self._log_ref) > 0:
            try:
                log = np.asarray([list(r)[:16] for r in self._log_ref], dtype=float)
            except Exception as e:
                print('[FigCapture] log rows not parseable:', e)
        if log is not None:
            n = min(T, log.shape[0])
            if log.shape[0] != T:
                print('[FigCapture] WARNING sidecar rows %d != log rows %d' % (T, log.shape[0]))
            if n > 0:
                t[:n] = log[:n, 0]
                if log.shape[1] >= 16:
                    collision[:n] = log[:n, 15].astype(np.int64)
        n_obs, n_pred, n_mpc = self.N_OBS, self.N_PRED, self.N_MPC
        ego = np.asarray(self._ego_rows, dtype=float).reshape(T, 4)
        obs = np.asarray(self._obs, dtype=float).reshape(T, n_obs, 4)
        obs_id = np.asarray(self._obs_id, dtype=np.int64).reshape(T, n_obs)
        pred_mean = np.asarray(self._pred, dtype=float).reshape(T, n_obs, n_pred, 3)
        pred_std = np.asarray(self._pred_std, dtype=float).reshape(T, n_obs)
        mpc_pred = np.asarray(self._mpc, dtype=float).reshape(T, n_mpc, 2)
        min_dist = np.asarray(self._min_dist, dtype=float).reshape(T)
        ref_raw0 = self._ref_raw0 if self._ref_raw0 is not None else np.zeros((0, 2))
        ref_blend0 = self._ref_blend0 if self._ref_blend0 is not None else np.zeros((0, 2))
        Tr = len(self._ref_t)
        Mmax = max([r.shape[0] for r in self._ref_xy], default=0)
        ref_blend_xy = np.full((Tr, Mmax, 2), np.nan)
        for i, r in enumerate(self._ref_xy):
            ref_blend_xy[i, :r.shape[0]] = r
        self._meta['n_rows'] = int(T)
        self._meta['n_frames'] = int(self._n_frames)
        self._meta['overview_done'] = bool(self._ov_saved)
        n = max(self._timer_n.get('capture_step', 0), 1)
        self._meta['timing_ms_per_step'] = {k: round(1000.0 * v / n, 3) for k, v in self._timers.items()}
        self._meta['frame_id_mismatches'] = int(self._frame_mismatch)
        os.makedirs(os.path.dirname(self._npz_path), exist_ok=True)
        np.savez(self._npz_path,
                 t=t, ego=ego, obs=obs, obs_id=obs_id,
                 pred_mean=pred_mean, pred_std=pred_std, mpc_pred=mpc_pred,
                 min_dist=min_dist, collision=collision,
                 ref_raw0=np.asarray(ref_raw0, dtype=float), ref_blend0=np.asarray(ref_blend0, dtype=float),
                 ref_blend_t=np.asarray(self._ref_t, dtype=float), ref_blend_xy=ref_blend_xy,
                 meta=json.dumps(self._meta))
        print('[FigCapture] sidecar written: %s rows=%d frames=%d overview=%s'
              % (self._npz_path, T, self._n_frames, self._ov_saved))
