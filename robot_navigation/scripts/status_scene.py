"""Task wording and field drawing shared by the GUI and offline verification.

The canvas only draws a configured field and odometry estimates; it does not
localize the robot or infer unobserved obstacles.
"""
import math
from collections import deque


def task_text(state, assignment=None, finished=None):
    assignment = assignment or {}
    second = state.endswith('_B')
    bed = assignment.get('second_bed' if second else 'first_bed')
    box = assignment.get('second_box' if second else 'first_box')
    target = ('%d号床' % bed) if bed in (1, 3) else ('第二张病床' if second else '第一张病床')
    medicine = ('%d号药箱' % box) if box in (1, 3) else '指定药箱'
    if state == 'STOP':
        return '任务完成，已回到起始区' if finished is True else '任务已停止'
    fixed = {
        'IDLE': '等待本体启动按钮', 'GOTO_NURSE': '正在前往护士台',
        'SCAN_QR': '已到护士台，正在等待二维码',
        'RETURN_HOME': '正在返回起始区',
        'HOME_CHECK': '已到起始区，检查整机入框并保持静止',
        'FAILED': '任务失败，车辆已停止', 'TIMEOUT': '任务超时，车辆已停止',
    }
    if state in fixed:
        return fixed[state]
    actions = {
        'GOTO_BED': '正在前往{bed}',
        'ARM_UPPER_ALIGN': '正在对准{bed}床头柜',
        'POSITION_IN_CIRCLE': '正在进行{bed}入圈微调',
        'SCAN_BARCODE': '已到{bed}，正在等待床头条形码',
        'OPEN_BOX': '正在为{bed}选择{box}',
        'PLACE_MEDICINE': '正在向{bed}放药（{box}），机械臂伸出、开箱及收回',
        'VOICE': '正在完成{bed}取药播报，底盘保持静止',
        'BEDSIDE': '正在与{bed}音视频交流并测量心率、体温',
    }
    key = state[:-2] if state.endswith(('_A', '_B')) else state
    return actions.get(key, state or '等待任务状态').format(bed=target, box=medicine)


class PoseTrack:
    def __init__(self, frame='odom', limit=600):
        self.frame = frame.strip('/')
        self.pose = None
        self.received = None
        self.problem = ''
        self.trail = deque(maxlen=limit)
        self.goal = None

    def update(self, frame, x, y, quaternion, now, age=0.0):
        if frame.strip('/') != self.frame:
            self.problem = '坐标系不匹配：%s，需要%s' % (frame or '空', self.frame)
            return False
        values = [x, y, *quaternion, now, age]
        if not all(math.isfinite(v) for v in values) or not -.05 <= age <= 1.0:
            self.problem = '定位数据无效或时间戳过期'
            return False
        norm = math.sqrt(sum(v*v for v in quaternion))
        if norm < 1e-6:
            self.problem = '定位朝向无效'
            return False
        qx, qy, qz, qw = [v/norm for v in quaternion]
        yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
        if self.pose and math.hypot(x-self.pose[0], y-self.pose[1]) > 1.0:
            # Do not join a localization reset with an imaginary driven segment.
            self.trail.clear()
        self.pose = (x, y, yaw)
        self.received, self.problem = now-max(0.0, age), ''
        if not self.trail or math.hypot(x-self.trail[-1][0], y-self.trail[-1][1]) >= .02:
            self.trail.append((x, y))
        return True

    def snapshot(self):
        return dict(pose=self.pose, received=self.received, problem=self.problem,
                    trail=list(self.trail), goal=self.goal)


def field_transform(width, height, bounds):
    xmin, xmax, ymin, ymax = bounds
    scale = min(max(1, width-40)/(ymax-ymin), max(1, height-40)/(xmax-xmin))
    left = (width-(ymax-ymin)*scale)/2
    top = (height-(xmax-xmin)*scale)/2
    # +x points up (toward nurse), +y points left in the arena view.
    return lambda x, y: (left+(ymax-y)*scale, top+(xmax-x)*scale), scale


def draw_field(canvas, width, height, scene, layout, now, timeout=1.0):
    """Draw on Tk Canvas; callers must execute on the GUI thread."""
    canvas.delete('all')
    bounds = layout.get('bounds', [-.5, 6.5, -3.5, 3.5])
    point, scale = field_transform(width, height, bounds)
    xmin, xmax, ymin, ymax = bounds
    def line(points, **options):
        canvas.create_line(*[n for p in points for n in point(*p)], **options)
    def rect(x1, x2, y1, y2, **options):
        a, b = point(x1, y1), point(x2, y2)
        canvas.create_rectangle(min(a[0],b[0]), min(a[1],b[1]),
                                max(a[0],b[0]), max(a[1],b[1]), **options)
    def label(x, y, text, **options):
        canvas.create_text(*point(x,y), text=text, fill='#cbd5e1',
                           font=('Microsoft YaHei', 9), **options)
    rect(xmin,xmax,ymin,ymax,fill='#111f2b',outline='#74879a')
    for x in range(math.ceil(xmin), math.floor(xmax)+1):
        line([(x,ymin),(x,ymax)],fill='#223647')
    for y in range(math.ceil(ymin), math.floor(ymax)+1):
        line([(xmin,y),(xmax,y)],fill='#223647')
    # Furniture rectangles are optional: only draw measured/configured objects.
    for item in layout.get('rectangles', []):
        rect(*item['bounds'], fill='#293b4b', outline='#526d82')
        x1,x2,y1,y2=item['bounds']
        label((x1+x2)/2,(y1+y2)/2,item['name'])
    for name, values in layout.get('stations', {}).items():
        x,y=values[:2]
        radius=(values[2] if len(values)>2 else .06)*scale
        cx,cy=point(x,y)
        canvas.create_oval(cx-radius,cy-radius,cx+radius,cy+radius,
                           outline='#72c8df',width=2)
        canvas.create_text(cx,cy+radius+11,text=name,fill='#d6e6ef',
                           font=('Microsoft YaHei',9))
    trail=scene.get('trail', [])
    if len(trail)>1:
        line(trail,fill='#3dada3',width=2)
    goal=scene.get('goal')
    if goal:
        gx,gy=point(*goal)
        canvas.create_line(gx-5,gy,gx+5,gy,fill='#eac768',width=2)
        canvas.create_line(gx,gy-5,gx,gy+5,fill='#eac768',width=2)
    pose=scene.get('pose')
    fresh=(pose is not None and scene.get('received') is not None
           and 0 <= now-scene['received'] <= timeout and not scene.get('problem'))
    if pose:
        x,y,yaw=pose
        cx,cy=point(x,y)
        color='#48d6bc' if fresh else '#edb65b'
        radius=max(4,layout.get('robot_radius_m',.25)*scale)
        canvas.create_oval(cx-radius,cy-radius,cx+radius,cy+radius,
                           outline=color,width=2)
        hx,hy=point(x+math.cos(yaw)*.4,y+math.sin(yaw)*.4)
        canvas.create_line(cx,cy,hx,hy,fill=color,width=3,arrow='last')
        inside=xmin <= x <= xmax and ymin <= y <= ymax
        status=('位置估计' if fresh else '定位已过期') if inside else '位置估计超出场地'
        status+='  x=%.2f  y=%.2f m  朝向=%.0f°' % (x,y,math.degrees(yaw))
    else:
        status='等待定位数据（不会假定车辆位于起点）'
    if scene.get('problem'):
        status=scene['problem']
    canvas.create_text(8,8,anchor='nw',text=status,fill='#dce8ed' if fresh else '#edb65b',
                       font=('Microsoft YaHei',9),width=max(80,width-16))
    canvas.create_text(8,height-6,anchor='sw',text='估计位置 · +x向上 · 绿线轨迹 · 黄十字目标',
                       fill='#a6bdcb',font=('Microsoft YaHei',8))
