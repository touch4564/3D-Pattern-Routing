# -*- coding: utf-8 -*-
"""

3D模式布线说明：

1. 起点引脚区域和终点引脚区域不在同一层：
   - 使用矩形区域到矩形区域的 3D L 型和 Z 型布线模式。

2. 起点引脚区域和终点引脚区域在同一层：
   - 使用矩形区域到矩形区域的 3D L 型和双 L 型布线模式。


"""
import math
import random
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TKagg')
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import plotly.graph_objects as go
import time
import re
from pattern_routing.Judge_the_violation import create_obstacle_hash
from pattern_routing.Optimize_line_segment import remove_overlaps_from_paths
from pattern_routing.Path_check import check_and_return_path_x,check_and_return_path_y, check_same_layer_x,\
check_same_layer_x_via, check_same_layer_y_via, check_path_all, check_path_samey_all, check_path_samex_all,\
check_node_via
# ------------------------- 参数定义 -------------------------
# 定义布线和通孔的各种参数，包括线宽、线距、通孔尺寸等
VIA_LENGTH = 9        # 通孔长度
VIA_WIDTH = 3.8       # 通孔宽度
rect_center = 2.6     # 矩形中心
VIA_LENGTH1 = 7        # 通孔长度
VIA_WIDTH1 = 3       # 通孔宽度
rect_center1 = 2     # 矩形中心

# 起点层线宽与线距
LINE_WIDTH_S = 2
LINE_SPACING_S = 2

# 终点层线宽与线距
LINE_WIDTH_T = 2
LINE_SPACING_T = 2

# 过滤距离计算
FILTER_DIS_S = LINE_WIDTH_S / 2 + LINE_SPACING_S
FILTER_DIS_T = LINE_WIDTH_T / 2 + LINE_SPACING_T

# 版图缩放因子
SCALE_FACTOR = 100

# ------------------------- 障碍物生成函数 -------------------------
def generate_random_rectangles(num_rectangles, min_width, max_width, min_height, max_height, layers, obstacles_to_avoid):
    """
    生成指定数量的不重叠矩形，随机分布到指定层，并避免与给定的矩形框重叠。

    参数:
    - num_rectangles: 需要生成的矩形数量
    - min_width: 矩形最小宽度
    - max_width: 矩形最大宽度
    - min_height: 矩形最小高度
    - max_height: 矩形最大高度
    - layers: 层列表
    - obstacles_to_avoid: 要避免重叠的矩形框列表

    返回:
    - 字典，键为层名称，值为该层的矩形列表
    """
    rectangles = {layer: [] for layer in layers}
    attempts = 0
    max_attempts = num_rectangles * 100  # 防止无限循环

    while sum(len(v) for v in rectangles.values()) < num_rectangles and attempts < max_attempts:
        attempts += 1
        width = random.randint(min_width, max_width)
        height = random.randint(min_height, max_height)
        x1 = random.randint(50, 450 - width)  # 限制在范围内
        y1 = random.randint(50, 450 - height)  # 限制在范围内
        x2 = x1 + width
        y2 = y1 + height
        new_rect = ((x1, y1), (x2, y2))

        # 检查新矩形是否与要避免的矩形框重叠
        overlap = False
        for obstacle1 in obstacles_to_avoid:
            obstacle = (obstacle1[0],obstacle1[1])
            if rectangles_overlap(new_rect, obstacle):
                overlap = True
                break

        if not overlap:
            # 随机选择一个层
            layer = random.choice(layers)

            # 检查新矩形是否与该层中已有的矩形重叠
            for rect in rectangles[layer]:
                if rectangles_overlap(new_rect, rect):
                    overlap = True
                    break

            if not overlap:
                rectangles[layer].append(new_rect)

    if attempts == max_attempts:
        print("达到最大尝试次数，可能无法生成所有不重叠的矩形。")

    return rectangles

def rectangles_overlap(rect1, rect2):
    """
    判断两个矩形是否重叠。

    参数:
    - rect1: 第一个矩形，格式为 ((x1_min, y1_min), (x1_max, y1_max))
    - rect2: 第二个矩形，格式为 ((x2_min, y2_min), (x2_max, y2_max))

    返回:
    - 重叠返回True，否则False
    """
    (x1_min, y1_min), (x1_max, y1_max) = rect1
    (x2_min, y2_min), (x2_max, y2_max) = rect2
    # 检查是否没有重叠
    if x1_max <= x2_min or x1_min >= x2_max or y1_max <= y2_min or y1_min >= y2_max:
        return False
    return True

# ------------------------- 路径规划函数 -------------------------
def Iteration_step_size(start_rect, end_rect, factor):
    """
    确定迭代的步长基于起始和终止矩形的尺寸。

    该方法根据起始矩形和终止矩形在x和y方向上的最大尺寸，
    计算适合的迭代步长。步长通过找到最大尺寸的位数，然后
    设定步长为该位数减二次方的结果（即10^(位数-2)）。
    这有助于在路径搜索过程中进行适当的采样和搜索步进。

    :return: 一个包含起始步长和终止步长的元组 (start_step, end_step)
    """
    # 计算起始矩形在x和y方向上的最大尺寸
    max_start = max(
        abs(start_rect[0][0] - start_rect[1][0]),
        abs(start_rect[0][1] - start_rect[1][1])
    )

    # 计算终止矩形在x和y方向上的最大尺寸
    max_end = max(
        abs(end_rect[0][0] - end_rect[1][0]),
        abs(end_rect[0][1] - end_rect[1][1])
    )

    # Initialize counters to determine the number of digits
    start_step = max(map_number_custom(max_start),math.ceil(LINE_WIDTH_S)) * factor
    end_step = max(map_number_custom(max_end),math.ceil(LINE_WIDTH_T)) * factor
    return start_step, end_step

def map_number_custom(x):
    """
    根据自定义的阈值和输出列表进行映射。

    该函数根据输入数值 `x` 的大小，将其映射到一个预先定义的输出值列表 `outputs` 中。
    映射的依据是数值范围的阈值列表 `thresholds`，即输入值 `x` 落在哪个阈值范围，
    就对应返回该范围的输出值。

    参数:
        x (float or int): 输入的数值，表示需要映射的值。
        thresholds (list): 定义数值范围的阈值列表，用于划分输入值的范围。
                           在函数中，固定为 [10, 100, 1000, 10000, 100000]。
        outputs (list): 对应阈值范围的输出值列表。
                        在函数中，固定为 [1, number, 10, 100, 1000, 5000]。

    返回:
        (int): 对应的输出值。如果输入值 `x` 超过所有阈值，则返回 `outputs` 列表中的最后一个值。
    """
    # 计算动态值 `number`：
    # - 计算 `(LINE_SPACING_T + LINE_WIDTH_T)` 和 `(LINE_SPACING_S + LINE_WIDTH_S)` 的和，
    #   然后取两者中的最小值。
    # - 使用 `math.ceil` 对最小值向上取整，得到 `number`。
    number = math.ceil(min((LINE_SPACING_T + LINE_WIDTH_T), (LINE_SPACING_S + LINE_WIDTH_S)))
    # 定义阈值列表，用于划分输入值范围。
    thresholds = [10, 100, 1000, 10000, 100000]

    # 定义输出值列表，对应每个阈值范围的输出值。
    # `number` 是动态计算的值，用于第二个范围（10 <= x < 100）。
    outputs = [1, number, number * 2, 100, 1000, 5000]

    # 遍历阈值列表，找到输入值 `x` 所属的范围，并返回对应的输出值。
    for i, threshold in enumerate(thresholds):
        if x <= threshold:
            # 如果 x 小于当前阈值，返回对应的输出值。
            return outputs[i]

    # 如果输入值 `x` 超过所有阈值，返回输出列表中的最后一个值。
    return outputs[-1]




def get_rectangle_access_points(rects, step):
    """
    从多个矩形中提取接入点，接入点是矩形边界上均匀间隔采样的点。

    参数:
        rects (list of tuples): 矩形列表，每个矩形格式为 ((x1, y1), (x2, y2), z)，
                                其中 (x1, y1) 和 (x2, y2) 是矩形的两个对角点，
                                z 是矩形所在的 z 坐标。
        step (int): 采样步长，定义在矩形边界上的采样间隔。

    返回:
        list of tuples: 接入点列表，每个点格式为 (x, y, z)，并确保没有重复的点。
    """
    points = []  # 存储接入点的列表

    for rect in rects:
        (x1, y1), (x2, y2), z = rect  # 提取矩形的对角点和 z 坐标

        # 确保 x1 <= x2 和 y1 <= y2，以便遍历时能正确采样
        x_min, x_max = sorted([x1, x2])
        y_min, y_max = sorted([y1, y2])

        # 采样下边界和上边界的接入点
        for x in range(math.ceil(x_min), int(x_max + 1), step):
            points.append((x, y_min, z))  # 添加下边界的点
            points.append((x, y_max, z))  # 添加上边界的点

        # 采样左边界和右边界的接入点
        for y in range(math.ceil(y_min),  int(y_max + 1), step):
            points.append((x_min, y, z))  # 添加左边界的点
            points.append((x_max, y, z))  # 添加右边界的点

    # 去重并返回接入点列表
    return list(set(points))  # 使用 set 去重，确保返回的点都是唯一的



def manhattan_distance_point_to_rect(point, rects):
    """
    计算三维点到多个矩形边界的最短曼哈顿距离。

    参数:
        point (tuple): 三维点的坐标，格式为 (x, y, z)。
        rects (list of tuples): 矩形列表，每个矩形由三个元素组成：
                                 ((x1, y1), (x2, y2), z_rect)。
                                 - (x1, y1) 和 (x2, y2) 是矩形对角线的两个顶点坐标。
                                 - z_rect 是矩形所在的 z 坐标。

    返回:
        int or float: 点到所有矩形边界的最短曼哈顿距离。
                       如果点位于某个矩形的边界上，则距离为 0。
    """
    min_total_dis = float('inf')  # 初始化最短距离为正无穷大

    x, y, z = point  # 分别提取点的 x, y, z 坐标

    for rect in rects:
        (x1, y1), (x2, y2), z_rect = rect  # 分别提取矩形的两个顶点坐标和 z 坐标

        # 如果点的 z 坐标与矩形的 z 坐标不同，跳过该矩形
        # # 因为曼哈顿距离需要在同一平面上计算
        # if z != z_rect:
        #     continue

        # 确保 x1 <= x2 和 y1 <= y2
        x_min, x_max = sorted([x1, x2])
        y_min, y_max = sorted([y1, y2])

        # 计算点在 x-y 平面上的曼哈顿距离到矩形边界
        if x_min <= x <= x_max and y_min <= y <= y_max:
            # 点位于矩形内部，距离为到最近边的距离
            distance_2d = min(x - x_min, x_max - x, y - y_min, y_max - y)
        elif x_min <= x <= x_max:
            # 点在 y 方向外部
            if y < y_min:
                distance_2d = y_min - y
            else:  # y > y_max
                distance_2d = y - y_max
        elif y_min <= y <= y_max:
            # 点在 x 方向外部
            if x < x_min:
                distance_2d = x_min - x
            else:  # x > x_max
                distance_2d = x - x_max
        else:
            # 点在 x 和 y 两个方向都在外部
            if x < x_min and y < y_min:
                distance_2d = (x_min - x) + (y_min - y)
            elif x < x_min and y > y_max:
                distance_2d = (x_min - x) + (y - y_max)
            elif x > x_max and y < y_min:
                distance_2d = (x - x_max) + (y_min - y)
            else:  # x > x_max 和 y > y_max
                distance_2d = (x - x_max) + (y - y_max)

        total_distance = distance_2d  # 曼哈顿距离仅考虑 x 和 y 方向

        # 更新最短距离和对应的点对
        if total_distance < min_total_dis:
            min_total_dis = total_distance

    return min_total_dis  # 返回点到所有矩形边界的最短曼哈顿距离



def compute_manhattan_distance(rect1, rect2):
    """
    计算两个矩形边缘之间的最短曼哈顿距离。

    参数:
    rect1: ((x1, y1), (x2, y2)) - 第一个矩形的两个对角线坐标
    rect2: ((x3, y3), (x4, y4)) - 第二个矩形的两个对角线坐标

    返回:
    最短的曼哈顿距离 (int)
    """

    # 提取第一个矩形的坐标
    x1_min, y1_min = min(rect1[0][0], rect1[1][0]), min(rect1[0][1], rect1[1][1])
    x1_max, y1_max = max(rect1[0][0], rect1[1][0]), max(rect1[0][1], rect1[1][1])

    # 提取第二个矩形的坐标
    x2_min, y2_min = min(rect2[0][0], rect2[1][0]), min(rect2[0][1], rect2[1][1])
    x2_max, y2_max = max(rect2[0][0], rect2[1][0]), max(rect2[0][1], rect2[1][1])

    # 计算x轴方向的最短距离
    if x1_max < x2_min:
        distance_x = x2_min - x1_max
    elif x2_max < x1_min:
        distance_x = x1_min - x2_max
    else:
        distance_x = 0  # x轴方向有重叠

    # 计算y轴方向的最短距离
    if y1_max < y2_min:
        distance_y = y2_min - y1_max
    elif y2_max < y1_min:
        distance_y = y1_min - y2_max
    else:
        distance_y = 0  # y轴方向有重叠

    # 曼哈顿距离是x和y方向距离的最大值
    manhattan_distance = max(distance_x , distance_y)

    return manhattan_distance

def compute_manhattan_distance_with_all_points(rect1, rect2):
    """
    计算两个矩形边缘之间的最短曼哈顿距离，并返回所有对应的点对。
    如果只有一对点满足最短曼哈顿距离，则返回这对点。

    参数:
    rect1: ((x1, y1), (x2, y2)) - 第一个矩形的两个对角线坐标
    rect2: ((x3, y3), (x4, y4)) - 第二个矩形的两个对角线坐标

    返回:
    (最短的曼哈顿距离 (int), 点对列表 (list of tuples))
    """

    # 提取第一个矩形的坐标
    x1_min, y1_min = math.ceil(min(rect1[0][0], rect1[1][0])),  math.ceil(min(rect1[0][1], rect1[1][1]))
    x1_max, y1_max = int(max(rect1[0][0], rect1[1][0])), int(max(rect1[0][1], rect1[1][1]))

    # 提取第二个矩形的坐标
    x2_min, y2_min = math.ceil(min(rect2[0][0], rect2[1][0])), math.ceil(min(rect2[0][1], rect2[1][1]))
    x2_max, y2_max = int(max(rect2[0][0], rect2[1][0])), int(max(rect2[0][1], rect2[1][1]))

    point_pairs = []
    min_distance = float('inf')

    # 遍历第一个矩形的所有边缘点
    rect1_points = []
    for x in range(x1_min, x1_max + 1, math.ceil(LINE_WIDTH_T + LINE_SPACING_T)):  # 步长改为 4
        rect1_points.append((x, y1_min))  # 下边
        rect1_points.append((x, y1_max))  # 上边
    for y in range(y1_min + 1, y1_max, math.ceil(LINE_WIDTH_T + LINE_SPACING_T)):  # 步长改为 4，避免重复上下边
        rect1_points.append((x1_min, y))  # 左边
        rect1_points.append((x1_max, y))  # 右边

    # 遍历第二个矩形的所有边缘点
    rect2_points = []
    for x in range(x2_min, x2_max + 1, math.ceil(LINE_WIDTH_T + LINE_SPACING_T)):  # 步长改为 4
        rect2_points.append((x, y2_min))  # 下边
        rect2_points.append((x, y2_max))  # 上边
    for y in range(y2_min + 1, y2_max, math.ceil(LINE_WIDTH_T + LINE_SPACING_T)):  # 步长改为 4，避免重复上下边
        rect2_points.append((x2_min, y))  # 左边
        rect2_points.append((x2_max, y))  # 右边

    # 计算所有点对的曼哈顿距离
    for p1 in rect1_points:
        for p2 in rect2_points:
            distance = abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])  # 曼哈顿距离公式
            if distance < min_distance:
                min_distance = distance
                point_pairs = [(p1, p2)]  # 更新最近点对
            elif distance == min_distance:
                point_pairs.append((p1, p2))  # 添加到最近点对列表

    return min_distance, point_pairs


def all_rects_point_pairs(s, t):
    """
    对于两个矩形列表 s 和 t，计算所有矩形对之间的最短曼哈顿距离，
    并返回达到该最短距离的所有点对列表。

    参数:
    s: list of rectangles - 每个矩形由两个对角线坐标表示 ((x1, y1), (x2, y2))
    t: list of rectangles - 同上

    返回:
    min_dis_point_pairs: 达到最短曼哈顿距离的所有点对列表，格式为 [((x1, y1), (x2, y2)), ...]
    """
    min_dis = float('inf')
    min_dis_point_pairs = []  # 初始化最短距离对应的点对列表为空

    for rect in s:
        for rect1 in t:
            # 计算两个矩形之间的最短曼哈顿距离和对应的点对列表
            dis, point_pairs = compute_manhattan_distance_with_all_points(rect, rect1)
            if dis < min_dis:
                # if len(point_pairs) > 1:
                #     min_dis = dis
                #     min_dis_point_pairs = point_pairs + min_dis_point_pairs
                # else:
                    min_dis = dis
                    # min_dis_point_pairs =  point_pairs + min_dis_point_pairs
                    min_dis_point_pairs = point_pairs
    return min_dis_point_pairs  # 返回所有达到最短距离的点对列表


def Dynamic_Step_Size(s, t):
    """
    动态确定路径规划中的步长。

    该函数基于起点和终点之间的最小曼哈顿距离来决定迭代步长，
    以优化路径搜索过程的效率和精度。

    参数:
    - s: 起始矩形，格式为 ((x1, y1), (x2, y2), z)
         表示一个三维矩形，其中 (x1, y1) 和 (x2, y2) 是矩形的两个对角点，z 表示高度。
    - t: 终止矩形，格式为 ((x1, y1), (x2, y2), z)
         格式与参数 s 相同。

    返回:
    - 一个整数，表示动态确定的步长大小。
      较大的步长适合远距离搜索（提高效率），较小的步长适合短距离搜索（提高精度）。
    """
    factor_1000 = 1
    # 计算起点和终点之间的最小曼哈顿距离:
    min_manhattan = compute_manhattan_distance(s, t)

    # 如果最小曼哈顿距离大于 1000，则认为点之间距离较远，可以使用较大的步长。
    if min_manhattan > 1000:
        return int(VIA_LENGTH + min(LINE_WIDTH_S, LINE_WIDTH_T)) * factor_1000

    # 取两者中的较大值，并向上取整作为步长。
    return int(LINE_WIDTH_S + LINE_SPACING_S)

def same_layer_pattern_routing(s, t, rects):
    """
    在同一层上进行模式布线，尝试3D L型和Z型模式路径连接。

    参数:
        s (tuple): 起始矩形，格式为 ((x1, y1), (x2, y2), z)
        t (tuple): 终止矩形，格式为 ((x1, y1), (x2, y2), z)
        rects (dict): 障碍物字典，格式为 {"layer_key": [((a, b), (c, d)), ...]}

    返回:
        tuple: 起点路径、终点路径、通孔列表
    """
    # 根据起点和终点确定迭代步长
    step = Dynamic_Step_Size(s[0], t[0])

    # 分类障碍物，仅考虑起点层的障碍物
    rect = []  # 存储起点层和终点层的障碍物
    rect1 = []  # 存储上一层的障碍物

    for key, value in rects.items():
        layer_num = int(re.search(r'\d+', key).group())  # 从层名称中提取层号
        if layer_num == 0:
            rect.extend(value)  # 将属于起点层的障碍物添加到 rect
        elif layer_num == 1 and key in rects:  # 检查键是否存在
            rect1.extend(value)  # 将起点层的上一层的障碍物添加到 rect1


    # 将分类后的障碍物列表转换为哈希表，以便快速查找和重叠检测


    rect = create_obstacle_hash(rect)  # 创建起点层障碍物的哈希表
    rect1 = create_obstacle_hash(rect1)  # 创建起点层障碍物的哈希表


    list_node = all_rects_point_pairs(s, t)

    # 确定模式布线步长
    s_step, t_step = Iteration_step_size(s[0], t[0], factor=2)

    # 获取起点和终点的接入点列表
    list_point_start = get_rectangle_access_points(s, s_step)
    list_point_end = get_rectangle_access_points(t, t_step)


    # 根据曼哈顿距离对起点和终点的接入点进行排序，以优化路径搜索顺序
    list_point_start_sorted = sorted(
        list_point_start,
        key=lambda point: manhattan_distance_point_to_rect(point, t)
    )
    list_point_end_sorted = sorted(
        list_point_end,
        key=lambda point: manhattan_distance_point_to_rect(point, s)
    )
    # 记录开始时间
    start_time = time.time()
    TIME_LIMIT = 5  # 时间限制（秒）

    # ------------------------- 多源多汇模式布线 ------------------------
    # 阶段1：尝试所有3D L型模式路径连接
    for (start_x, start_y, z_start) in list_point_start_sorted:
        for (end_x, end_y, z_end) in list_point_end_sorted:

            # 尝试沿X方向优先的L型路径布线
            s_path, t_path, vias = check_same_layer_x(start_x, start_y, start_x, end_x, end_y, rect)
            if s_path is not None:
                return s_path, [], []

            # 尝试另一种X方向优先的L型路径布线
            s_path, t_path, vias = check_same_layer_x(
                start_x, start_y, end_x, end_x, end_y, rect)
            if s_path is not None:
                return s_path, [], []

    # 阶段2：根据匹配点得到双孔的模式布线
    dict_node={} #避免重复搜索
    for i in range(0, len(list_node), math.ceil(LINE_WIDTH_S + LINE_SPACING_S)):
        (nodes, nodet) = list_node[i]
        start_x, start_y = nodes
        end_x, end_y = nodet
        if start_y == end_y:
            for x in range(math.ceil(min(start_x , end_x)) , max(start_x,end_x),  step):
                for x1 in range(math.ceil(x + math.ceil(VIA_LENGTH + LINE_SPACING_S)), int(max(start_x,end_x) +  step),   step):
                    s_path, t_path, vias = check_path_samey_all(start_x, start_y, x, x1, end_x, end_y, rect, rect1)
                    if s_path is not None:
                        return s_path, t_path, vias

        elif start_x == end_x:
            for y in range(math.ceil(min(start_y, end_y)) , max(start_y, end_y),  step):
                for y1 in range(math.ceil(y + math.ceil(VIA_WIDTH + LINE_SPACING_S)) , int(max(start_y, end_y) +  step),   step):
                    s_path, t_path, vias = check_path_samex_all(start_x, start_y, y, y1, end_x, end_y, rect, rect1)

                    if s_path is not None:
                        return s_path, t_path, vias
        else:
            # 阶段2：如果阶段1未找到路径，进行基于迭代步长的3D 双L模式路径搜索
            # 迭代步长搜索路径
            y_range = range(math.ceil(start_y) , int(end_y) +  step, step) if math.ceil(start_y) + step + 1 < end_y else range(math.ceil(end_y) ,
                                                                                                  int(start_y) + step, step)
            x_range = range(math.ceil(start_x) , int(end_x) +  step, step) if math.ceil(start_x) + step + 1 < end_x else range(math.ceil(end_x) ,
                                                                                                  int(start_x) + step, step)
            for x in x_range:
                for y in y_range:
                    s_path, t_path, vias, dict_node = check_path_all(start_x, start_y, x, y, end_x, end_y, rect, rect1, dict_node)

                    if s_path is not None:
                        return s_path, t_path, vias

    # 阶段3：如果阶段2未找到路径，进行基于迭代步长的3D Z型模式路径搜索
    for (start_x, start_y, z_start) in list_point_start_sorted:
        for (end_x, end_y, z_end) in list_point_end_sorted:
            if ((start_x, start_y),(end_x, end_y)) in list_node:
                continue
            current_time = time.time()
            if current_time - start_time > TIME_LIMIT:
                # print("遍历时间超过十秒，返回空结果。")
                return [], [], []

            x_range = range(math.ceil(start_x) , int(end_x) + step, step) if math.ceil(start_x) + step + 1 < end_x else range(math.ceil(end_x) ,
                                                                                                  int(start_x) + step, step)
            # 尝试沿X轴迭代步长搜索路径
            for x in x_range:
                s_path, t_path, vias = check_same_layer_x_via(
                    start_x, start_y, x, end_x, end_y, rect, rect1
                )
                if s_path is not None:
                    return s_path, t_path, vias

            # 尝试沿Y轴迭代步长搜索路径
            y_range = range(math.ceil(start_y) , int(end_y) + step, int(step)) if math.ceil(start_y + step) + 1 < end_y else range(math.ceil(end_y ) , int(start_y) + step, int(step))
            for y in y_range:
                s_path, t_path, vias = check_same_layer_y_via(
                    start_x, start_y, y, end_x, end_y, rect, rect1
                )
                if s_path is not None:
                    return s_path, t_path, vias

    # 若未找到可行路径，返回空结果
    return [], [], []

def manhattan_distance(point1, point2):
    """
    计算两个点之间的曼哈顿距离。

    参数:
    point1 (tuple): 第一个点，格式为 (x1, y1) 或 (x1, y1, z1) 等。
    point2 (tuple): 第二个点，格式为 (x2, y2) 或 (x2, y2, z2) 等。

    返回:
    float: 两个点之间的曼哈顿距离。
    """
    # 确保两个点的维度相同
    if len(point1) != len(point2):
        raise ValueError("两个点的维度必须相同")

    # 计算曼哈顿距离
    distance = sum(abs(a - b) for a, b in zip(point1, point2))
    return distance

def has_valid_distance(vias, vias1):
    """
    检查 vias 和 vias1 中的点之间是否存在符合条件的曼哈顿距离。

    参数:
    vias (list): 第一个点的列表，格式为 [((x1, y1),), ((x2, y2),), ...]
    vias1 (list): 第二个点的列表，格式为 [((x1, y1),), ((x2, y2),), ...]
    via_length (float): 定义的 VIA_LENGTH
    line_spacing_s (float): 定义的 LINE_SPACING_S

    返回:
    bool: 如果存在符合条件的距离，则返回 True，否则返回 False。
    """
    for via in vias:
        for via1 in vias1:
            if 0 < manhattan_distance(via1[0], via[0]) < VIA_LENGTH + LINE_SPACING_S:
                return True  # 找到符合条件的距离
    return False  # 没有找到符合条件的距离


def same_layer_pattern_routing_three_layers(s, t, rects):
    """
    在同一层上进行模式布线，尝试3D L型和Z型模式路径连接。

    参数:
        s (tuple): 起始矩形，格式为 ((x1, y1), (x2, y2), z)
        t (tuple): 终止矩形，格式为 ((x1, y1), (x2, y2), z)
        rects (dict): 障碍物字典，格式为 {"layer_key": [((a, b), (c, d)), ...]}

    返回:
        tuple: 起点路径、终点路径、通孔列表
    """
    # 根据起点和终点确定迭代步长
    step = Dynamic_Step_Size(s[0], t[0])

    # 分类障碍物，仅考虑起点层的障碍物
    rect = []  # 存储起点层和终点层的障碍物
    rect1 = []  # 存储上一层的障碍物
    rect2 = []  # 存储上上一层的障碍物


    for key, value in rects.items():
        layer_num = int(re.search(r'\d+', key).group())  # 从层名称中提取层号
        if layer_num == 0:
            rect.extend(value)  # 将属于起点层的障碍物添加到 rect
        elif layer_num == 1 and key in rects:  # 检查键是否存在
            rect1.extend(value)  # 将起点层的上一层的障碍物添加到 rect1
        elif layer_num == 2 and key in rects:  # 检查键是否存在
            rect2.extend(value)  # 将起点层的上一层的障碍物添加到 rect1

    # 将分类后的障碍物列表转换为哈希表，以便快速查找和重叠检测


    rect = create_obstacle_hash(rect)  # 创建起点层障碍物的哈希表
    rect1 = create_obstacle_hash(rect1)  # 创建起点层障碍物的哈希表
    rect2 = create_obstacle_hash(rect2)  # 创建起点层障碍物的哈希表


    list_node = all_rects_point_pairs(s, t)

    # 确定模式布线步长
    s_step, t_step = Iteration_step_size(s[0], t[0], factor=1)

    # 获取起点和终点的接入点列表
    list_point_start = get_rectangle_access_points(s, s_step)
    list_point_end = get_rectangle_access_points(t, t_step)


    # 根据曼哈顿距离对起点和终点的接入点进行排序，以优化路径搜索顺序
    list_point_start_sorted = sorted(
        list_point_start,
        key=lambda point: manhattan_distance_point_to_rect(point, t)
    )
    list_point_end_sorted = sorted(
        list_point_end,
        key=lambda point: manhattan_distance_point_to_rect(point, s)
    )
    # 记录开始时间
    start_time = time.time()
    TIME_LIMIT = 5  # 时间限制（秒）
    # ------------------------- 多源多汇模式布线 ------------------------
    # 阶段1：尝试所有3D L型模式路径连接
    for (start_x, start_y, z_start) in list_point_start_sorted:
        for (end_x, end_y, z_end) in list_point_end_sorted:
            vias1 = check_node_via((start_x, start_y), (end_x, end_y), rect,rect1)
            if vias1 is not None:
                # 尝试沿X方向优先的L型路径布线
                s_path, t_path, vias = check_same_layer_x(start_x, start_y, start_x, end_x, end_y, rect1)
                if s_path is not None:
                    return s_path, [], [] , vias1

                # 尝试另一种X方向优先的L型路径布线
                s_path, t_path, vias = check_same_layer_x(
                    start_x, start_y, end_x, end_x, end_y, rect1)
                if s_path is not None:
                    return s_path, [], [] ,vias1

    # 阶段2：根据匹配点得到双孔的模式布线
    dict_node={} #避免重复搜索
    for i in range(0, len(list_node), math.ceil(LINE_WIDTH_S + LINE_SPACING_S)):
        (nodes, nodet) = list_node[i]
        start_x, start_y = nodes
        end_x, end_y = nodet
        vias1 = check_node_via((start_x, start_y), (end_x, end_y), rect,rect1)
        if vias1 is not None:
            if start_y == end_y:
                for x in range(math.ceil(min(start_x , end_x)) , max(start_x,end_x),  step):
                    if 0 < abs(x - start_x) < VIA_LENGTH + LINE_SPACING_S or 0 < abs(x - end_x) < VIA_LENGTH + LINE_SPACING_S:
                        continue

                    for x1 in range(math.ceil(x + math.ceil(VIA_LENGTH + LINE_SPACING_S)) , int(max(start_x,end_x) +  step),   step):
                        if 0< abs(x1 - start_x) < VIA_LENGTH + LINE_SPACING_S or \
                                 0 < abs(x1 - end_x) < VIA_LENGTH + LINE_SPACING_S:
                            continue
                        s_path, t_path, vias = check_path_samey_all(start_x, start_y, x, x1, end_x, end_y, rect1, rect2)
                        if s_path is not None:
                            return s_path, t_path, vias, vias1

            elif start_x == end_x:
                for y in range(math.ceil(min(start_y, end_y)) , max(start_y, end_y),  step):
                    for y1 in range(math.ceil(y + math.ceil(VIA_WIDTH + LINE_SPACING_S)) , int(max(start_y, end_y) +  step),   step):
                        if 0 < abs(y - start_y) < VIA_LENGTH + LINE_SPACING_S or 0 < abs(
                                y1 - start_y) < VIA_LENGTH + LINE_SPACING_S or \
                                0 < abs(y - end_y) < VIA_LENGTH + LINE_SPACING_S or 0 < abs(
                            y1 - end_y) < VIA_LENGTH + LINE_SPACING_S:
                            continue
                        s_path, t_path, vias = check_path_samex_all(start_x, start_y, y, y1, end_x, end_y, rect1, rect2)

                        if s_path is not None:
                            return s_path, t_path, vias, vias1
            else:
                # 阶段2：如果阶段1未找到路径，进行基于迭代步长的3D 双L模式路径搜索
                # 迭代步长搜索路径
                y_range = range(math.ceil(start_y) , int(end_y) +  step, step) if math.ceil(start_y) + step + 1 < end_y else range(math.ceil(end_y),
                                                                                                      int(start_y) + step, step)
                x_range = range(math.ceil(start_x) , int(end_x) +  step, step) if math.ceil(start_x) + step + 1 < end_x else range(math.ceil(end_x),
                                                                                                      int(start_x) + step, step)
                for x in x_range:
                    for y in y_range:
                        s_path, t_path, vias, dict_node = check_path_all(start_x, start_y, x, y, end_x, end_y, rect1, rect2, dict_node)

                        if s_path is not None:
                            if not has_valid_distance(vias, vias1):
                                return s_path, t_path, vias, vias1

    # 阶段3：如果阶段2未找到路径，进行基于迭代步长的3D Z型模式路径搜索
    for (start_x, start_y, z_start) in list_point_start_sorted:
        for (end_x, end_y, z_end) in list_point_end_sorted:
            if ((start_x, start_y),(end_x, end_y)) in list_node:
                continue
            vias1 = check_node_via((start_x, start_y), (end_x, end_y), rect,rect1)
            if vias1 is not None:
                current_time = time.time()
                if current_time - start_time > TIME_LIMIT:
                    # print("遍历时间超过十秒，返回空结果。")
                    return [], [], [], []

                x_range = range(math.ceil(start_x) , int(end_x) + step, step) if math.ceil(start_x) + step + 1 < end_x else range(math.ceil(end_x),
                                                                                                      int(start_x) + step, step)
                # 尝试沿X轴迭代步长搜索路径
                for x in x_range:
                    s_path, t_path, vias = check_same_layer_x_via(
                        start_x, start_y, x, end_x, end_y, rect1, rect2
                    )
                    if s_path is not None:
                        if not has_valid_distance(vias, vias1):
                            return s_path, t_path, vias, vias1


                # 尝试沿Y轴迭代步长搜索路径
                y_range = range(math.ceil(start_y), int(end_y) + step, int(step)) if math.ceil(start_y + step) + 1 < end_y else range(math.ceil(end_y ), int(start_y) + step, int(step))
                for y in y_range:
                    s_path, t_path, vias = check_same_layer_y_via(
                        start_x, start_y, y, end_x, end_y, rect1, rect2
                    )
                    if s_path is not None:
                        if not has_valid_distance(vias, vias1):
                            return s_path, t_path, vias, vias1

    # 若未找到可行路径，返回空结果
    return [], [], [], []


# ------------------------- 不同层路径规划函数 -------------------------
def different_layers_pattern_routing_(s, t, rects):
    """
    利用3D Z型模式进行布线。

    参数:
    - s: 起始矩形，格式为 ((x1, y1), (x2, y2), z)
    - t: 目标矩形，格式为 ((x1, y1), (x2, y2), z)
    - rects: 障碍物字典，格式为 {"layer_key": [((a, b), (c, d)), ...]}

    返回:
    - 起点路径, 终点路径, 通孔列表
    """
    # 迭代步长由距离确定
    step = Dynamic_Step_Size(s[0], t[0])

    # # 计算起始和目标矩形的中心点
    # s_center = ((s[0][0] + s[1][0]) // 2, (s[0][1] + s[1][1]) // 2)
    # t_center = ((t[0][0] + t[1][0]) // 2, (t[0][1] + t[1][1]) // 2)
    #
    # # 确保起点在左侧
    # if s_center[0] > t_center[0]:
    #     s, t = t, s

    # 分类障碍物
    rect_s = []  # 起点层障碍物
    rect_t = []  # 终点层障碍物
    rect_via = []  # 所有层通孔障碍物



    for key, value in rects.items():
        layer_num = int(re.search(r'\d+', key).group())  # 从层名称中提取层号
        if layer_num == s[0][2]:
            rect_s.extend(value)  # 将属于起点层的障碍物添加到 rect_s
            rect_via.extend(value)  # 通孔障碍物包含起点层的障碍物
        elif layer_num == t[0][2]:
            rect_t.extend(value)  # 将属于终点层的障碍物添加到 rect_t
            rect_via.extend(value)  # 通孔障碍物包含终点层的障碍物
        elif min(s[0][2],t[0][2]) < layer_num and layer_num < max(s[0][2],t[0][2]):
            rect_via.extend(value)  # 将其他层的障碍物添加到 rect_via

    # 将分类后的障碍物列表转换为哈希表（字典），以便快速查找和重叠检测
    rect_s = create_obstacle_hash(rect_s)  # 创建起点层障碍物的哈希表
    rect_t = create_obstacle_hash(rect_t)  # 创建终点层障碍物的哈希表
    rect_via = create_obstacle_hash(rect_via)  # 创建所有层通孔障碍物的哈希表

    # 确定模式布线步长
    s_step, t_step = Iteration_step_size(s[0], t[0], factor=2)

    # 获取起点和终点的接入点列表
    list_point_start = get_rectangle_access_points(s, s_step)
    list_point_end = get_rectangle_access_points(t, t_step)

    # 根据曼哈顿距离对起点和终点的接入点进行排序，以优化路径搜索顺序
    list_point_start_sorted = sorted(
        list_point_start,
        key=lambda point: manhattan_distance_point_to_rect(point, t)
    )

    list_point_end_sorted = sorted(
        list_point_end,
        key=lambda point: manhattan_distance_point_to_rect(point, s)
    )

    # 记录开始时间
    start_time = time.time()
    TIME_LIMIT = 5  # 时间限制（秒）

    # 阶段1：尝试所有 3D L型模式路径连接
    for (start_x, start_y, z_start) in list_point_start_sorted:
        for (end_x, end_y, z_end) in list_point_end_sorted:

            # 尝试x方向L型模式布线优先路径
            s_path, t_path, vias = check_and_return_path_x(
                start_x, start_y, start_x, end_x, end_y, rect_s, rect_t, rect_via
            )
            if s_path is not None:
                return s_path, t_path, vias

            s_path, t_path, vias = check_and_return_path_x(
                start_x, start_y, end_x, end_x, end_y, rect_s, rect_t, rect_via
            )
            if s_path is not None:
                return s_path, t_path, vias

            # 尝试y方向L型模式布线优先路径
            s_path, t_path, vias = check_and_return_path_y(
                start_x, start_y, start_y, end_x, end_y, rect_s, rect_t, rect_via
            )
            if s_path is not None:
                return s_path, t_path, vias

            s_path, t_path, vias = check_and_return_path_y(
                start_x, start_y, end_y, end_x, end_y, rect_s, rect_t, rect_via
            )
            if s_path is not None:
                return s_path, t_path, vias

    # 阶段2：如果阶段1未找到路径，进行基于迭代步长的 3D Z型模式路径搜索
    for (start_x, start_y, z_start) in list_point_start_sorted:
        for (end_x, end_y, z_end) in list_point_end_sorted:

            current_time = time.time()
            if current_time - start_time > TIME_LIMIT:
                # print("遍历时间超过十秒，返回空结果。")
                return [], [], []

            # 尝试沿 x 轴迭代步长搜索路径
            for x in range(math.ceil(start_x + step), int(end_x), step):
                current_time = time.time()
                if current_time - start_time > TIME_LIMIT:
                    # print("遍历时间超过十秒，返回空结果。")
                    return [], [], []

                s_path, t_path, vias = check_and_return_path_x(
                    start_x, start_y, x, end_x, end_y, rect_s, rect_t, rect_via
                )
                if s_path is not None:
                    return s_path, t_path, vias


            # 尝试沿 y 轴迭代步长搜索路径
            y_range = range(math.ceil(start_y + step), int(end_y), step) if start_y + step + 1 < end_y else range(math.ceil(end_y + step), int(start_y), step)
            for y in y_range:
                current_time = time.time()
                if current_time - start_time > TIME_LIMIT:
                    # print("遍历时间超过十秒，返回空结果。")
                    return [], [], []

                s_path, t_path, vias = check_and_return_path_y(
                    start_x, start_y, y, end_x, end_y, rect_s, rect_t, rect_via
                )
                if s_path is not None:
                    return s_path, t_path, vias
    # 若未找到可行路径，返回空
    return [], [], []


# ------------------------- 路径规划核心函数 -------------------------
def find_path(s, t, rects):
    """

    根据起点和终点是否在同一层，调用相应的布线函数进行路径规划。

    """
    if s[0][2] == t[0][2]:
        # 如果起点和终点在同一层，调用同层布线函数
        s_path, t_path, vias = same_layer_pattern_routing(s, t, rects)

    else:
        # 如果起点和终点不在同一层，调用不同层布线函数
        s_path, t_path, vias = different_layers_pattern_routing_(s, t, rects)

    for s1 in s:
        for t1 in t:
            # 移除路径中与起点和终点重叠的部分
            s_path, t_path = remove_overlaps_from_paths(s_path, t_path, s1, t1)
    return s_path, t_path, vias



def Pin_grouping(s, t):
    """
    将输入的两个列表 s 和 t 中的元素进行分组，并移除具有特定标记的元素。

    参数:
    s (list): 包含多个元素的列表，每个元素是一个元组或列表，要求其包含至少 3 个元素。
    t (list): 另一个包含多个元素的列表，每个元素也是一个元组或列表，要求其包含至少 3 个元素。

    返回:
    tuple: 返回两个列表，分别是 s 和 t 中所有第三个值为 1 的元素 s_1 和 t_1。
    """

    s_1 = []  # 初始化列表，保存从 s 中筛选出来的元素
    t_1 = []  # 初始化列表，保存从 t 中筛选出来的元素

    # 遍历列表 s，筛选出第三个元素值为 1 的元素并添加到 s_1 列表
    for s1 in s:
        if s1[2] == 1:  # 检查元素的第三个值
            s_1.append(s1)  # 将符合条件的元素添加到 s_1 列表

    # 遍历列表 t，筛选出第三个元素值为 1 的元素并添加到 t_1 列表
    for t1 in t:
        if t1[2] == 1:  # 检查元素的第三个值
            t_1.append(t1)  # 将符合条件的元素添加到 t_1 列表

    # 从原列表中移除已经添加到 s_1 的元素
    remove_elements_in_place(s, s_1)  # 在列表 s 中移除所有出现在 s_1 中的元素

    # 从原列表中移除已经添加到 t_1 的元素
    remove_elements_in_place(t, t_1)  # 在列表 t 中移除所有出现在 t_1 中的元素

    return s_1, t_1  # 返回筛选出的元素列表


def remove_elements_in_place(list_a, list_b):
    """
    从 list_a 中移除 list_b 中的所有元素。

    参数:
    list_a (list): 要修改的列表
    list_b (list): 包含要移除元素的列表

    返回:
    None: 该函数会直接修改 list_a 列表，不返回任何值。
    """
    for item in list_b:
        # 使用 while 循环确保所有出现的 item 都被移除
        while item in list_a:
            list_a.remove(item)


def find_path_cross_layers(s, t, rects):
    """

    根据起点和终点是否在同一层，调用相应的布线函数进行路径规划。

    """
    # 如果起点和终点在同一层，调用同层布线函数
    s1, t1 = Pin_grouping(s, t)
    # 如果起点和终点在同一层，调用同层布线函数
    s_path, t_path, vias, vias1= same_layer_pattern_routing_three_layers(s, t, rects)
    if s_path == [] and t_path == [] and vias == [] and s1 != []:
        s_path, t_path, vias1= different_layers_pattern_routing_(s1, t, rects)
    if s_path == [] and t_path == [] and vias == [] and t1 != []:
        s_path, t_path, vias1 = different_layers_pattern_routing_(s, t1, rects)
    return s_path, t_path, vias, vias1

# ------------------------- 3D可视化函数 -------------------------
# def visualize_routing(s, t, s_path, t_path, res_via, obstacle_rects):
#     """
#     3D 可视化布线路径、通孔、障碍物、起点和终点的矩形框。
#
#     参数:
#     - s: 起始矩形，格式为 ((x1, y1), (x2, y2), z)
#     - t: 目标矩形，格式为 ((x1, y1), (x2, y2), z)
#     - s_path: 起点层路径列表，格式为 [((x1, y1), (x2, y2)), ...]
#     - t_path: 终点层路径列表，格式为 [((x1, y1), (x2, y2)), ...]
#     - res_via: 通孔列表，格式为 [( (x, y), layer ), ...]
#     - obstacle_rects: 障碍物字典，格式为 {"layer_key": [((a,b),(c,d)), ...]}
#     """
#     fig = plt.figure(figsize=(12, 10))
#     ax = fig.add_subplot(111, projection='3d')
#
#     # 获取所有层数，包括起点和终点所在层
#     layers = sorted(set(int(re.search(r'\d+', layer).group()) for layer in obstacle_rects.keys()))
#     layers += [s[2], t[2]]  # 确保起点和终点层数包含在内
#     layers = sorted(set(layers))  # 去重并排序
#
#     layer_heights = {layer: layer for layer in layers}  # 定义层高度
#
#
#     # 动态分配颜色：若层数超出预设，自动生成颜色
#     base_colors = [ '#505050','black', '#008000', 'blue', 'purple', 'orange', 'cyan', 'magenta']
#     colors = {layer: base_colors[i % len(base_colors)] for i, layer in enumerate(layers)}
#
#     # 绘制障碍物
#     for layer, rects in obstacle_rects.items():
#         z = layer_heights[int(re.search(r'\d+', layer).group())]  # 获取层高度
#         for (x1, y1), (x2, y2) in rects:
#             # 定义矩形的四个角
#             corners = [
#                 (x1, y1, z),
#                 (x2, y1, z),
#                 (x2, y2, z),
#                 (x1, y2, z)
#             ]
#             # 创建多边形
#             poly = Poly3DCollection([corners], alpha=0.5)
#             poly.set_color(colors[int(re.search(r'\d+', layer).group())])
#             ax.add_collection3d(poly)
#
#     # 绘制起点矩形框
#     s_z = layer_heights[s[2]]
#     s_x1, s_y1 = s[0]
#     s_x2, s_y2 = s[1]
#     s_corners = [
#         (s_x1, s_y1, s_z),
#         (s_x2, s_y1, s_z),
#         (s_x2, s_y2, s_z),
#         (s_x1, s_y2, s_z)
#     ]
#     s_poly = Poly3DCollection([s_corners], alpha=0.3)
#     s_poly.set_color('green')  # 起点使用绿色
#     ax.add_collection3d(s_poly)
#     # 添加起点标签
#     ax.text((s_x1 + s_x2)/2, (s_y1 + s_y2)/2, s_z, 'Start', color='green', fontsize=12, ha='center', va='center')
#
#     # 绘制终点矩形框
#     t_z = layer_heights[t[2]]
#     t_x1, t_y1 = t[0]
#     t_x2, t_y2 = t[1]
#     t_corners = [
#         (t_x1, t_y1, t_z),
#         (t_x2, t_y1, t_z),
#         (t_x2, t_y2, t_z),
#         (t_x1, t_y2, t_z)
#     ]
#     t_poly = Poly3DCollection([t_corners], alpha=0.3)
#     t_poly.set_color('red')  # 终点使用红色
#     ax.add_collection3d(t_poly)
#     # 添加终点标签
#     ax.text((t_x1 + t_x2)/2, (t_y1 + t_y2)/2, t_z, 'End', color='red', fontsize=12, ha='center', va='center')
#
#     # 绘制s_path（起点层路径）
#     for (start, end) in s_path:
#         xs = [start[0], end[0]]
#         ys = [start[1], end[1]]
#         zs = [s_z, s_z]  # 使用起点的层高度
#         ax.plot(xs, ys, zs, color='red', linewidth=2, label='s_path' if 's_path' not in ax.get_legend_handles_labels()[1] else "")
#
#     # 绘制t_path（终点层路径）
#     for (start, end) in t_path:
#         xs = [start[0], end[0]]
#         ys = [start[1], end[1]]
#         zs = [t_z + 1, t_z + 1] if s[2] == t[2] else [t_z, t_z]
#         ax.plot(xs, ys, zs, color='blue', linewidth=2, label='t_path' if 't_path' not in ax.get_legend_handles_labels()[1] else "")
#
#     # 绘制通孔（从起点层到终点层）
#     for (coord, layer) in res_via:
#         x, y = coord
#         z_start = layer_heights[s[2]]
#         z_end = layer_heights[t[2]] + 1 if s[2] == t[2] else layer_heights[t[2]]
#         ax.plot([x, x], [y, y], [z_start, z_end], color='black', linewidth=2, label='Vias' if 'Vias' not in ax.get_legend_handles_labels()[1] else "")
#
#     # 设置坐标轴标签
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Layer')
#
#     # 设置坐标轴范围
#     all_x = [coord for segment in s_path + t_path for coord in [segment[0][0], segment[1][0]]]
#     all_y = [coord for segment in s_path + t_path for coord in [segment[0][1], segment[1][1]]]
#     all_x += [via[0] for via, _ in res_via]
#     all_y += [via[1] for via, _ in res_via]
#     all_x += [s_x1, s_x2, t_x1, t_x2]  # 包含起点和终点的坐标
#     all_y += [s_y1, s_y2, t_y1, t_y2]
#     ax.set_xlim(min(all_x) - 500, max(all_x) + 500)
#     ax.set_ylim(min(all_y) - 500, max(all_y) + 500)
#     ax.set_zlim(-1, max(layers) + 2)  # 根据最大层数设置 z 轴范围
#
#     # 添加图例
#     ax.legend()
#
#     # 添加标题
#     plt.title("3D pattern Routing Visualization")
#     # 显示图形
#     plt.show()
def visualize_routing(s1, t1, s_path, t_path, res_via, obstacle_rects):
    """
    3D 可视化布线路径、通孔、障碍物、起点和终点的矩形框。

    参数:
    - s: 起始矩形，格式为 ((x1, y1), (x2, y2), z)
    - t: 目标矩形，格式为 ((x1, y1), (x2, y2), z)
    - s_path: 起点层路径列表，格式为 [((x1, y1), (x2, y2)), ...]
    - t_path: 终点层路径列表，格式为 [((x1, y1), (x2, y2)), ...]
    - res_via: 通孔列表，格式为 [( (x, y), layer ), ...]
    - obstacle_rects: 障碍物字典，格式为 {"layer_key": [((a,b),(c,d)), ...]}
    """
    # 创建图形
    fig = go.Figure()

    # 获取所有层数，包括起点和终点所在层
    layers = sorted(set(int(re.search(r'\d+', layer).group()) for layer in obstacle_rects.keys()))
    layers += [s1[0][2], t1[0][2]]  # 确保起点和终点层数包含在内
    layers = sorted(set(layers))  # 去重并排序

    layer_heights = {layer: layer for layer in layers}  # 定义层高度

    # 定义不同层的颜色
    color_map = {
        0: 'rgba(0, 0, 0, 1)',      # 第一层颜色：黑色
        1: 'rgba(50, 50, 50, 0.8)',   # 第二层颜色：深灰色
        # 可以添加更多层的颜色
        2: 'blue',
        3: 'purple'
    }

    # 绘制障碍物
    for layer, rects in obstacle_rects.items():
        layer_num = int(re.search(r'\d+', layer).group())  # 获取层数
        z = layer_heights[layer_num]  # 获取层高度
        for (x1, y1), (x2, y2) in rects:
            # 定义矩形的四个角
            corners = [
                (x1, y1, z),
                (x2, y1, z),
                (x2, y2, z),
                (x1, y2, z)
            ]
            # 创建多边形
            fig.add_trace(go.Mesh3d(
                x=[corner[0] for corner in corners],
                y=[corner[1] for corner in corners],
                z=[corner[2] for corner in corners],
                opacity=0.5,
                color=color_map.get(layer_num, 'grey')  # 使用层对应的颜色，默认为灰色
            ))
    for s in s1:
        # 绘制起点矩形框
        s_x1, s_y1 = s[0]
        s_x2, s_y2 = s[1]
        s_z = layer_heights[s[2]]
        s_corners = [
            (s_x1, s_y1, s_z),
            (s_x2, s_y1, s_z),
            (s_x2, s_y2, s_z),
            (s_x1, s_y2, s_z)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in s_corners],
            y=[corner[1] for corner in s_corners],
            z=[corner[2] for corner in s_corners],
            opacity=0.8,
            color='red'
        ))
        fig.add_annotation(x=(s_x1 + s_x2) / 2, y=(s_y1 + s_y2) / 2,
                           text="Start", showarrow=True, arrowhead=2, ax=0, ay=-40, font=dict(color='green'))
    for t in t1:
        # 绘制终点矩形框
        t_x1, t_y1 = t[0]
        t_x2, t_y2 = t[1]
        t_z = layer_heights[t[2]]
        t_corners = [
            (t_x1, t_y1, t_z),
            (t_x2, t_y1, t_z),
            (t_x2, t_y2, t_z),
            (t_x1, t_y2, t_z)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in t_corners],
            y=[corner[1] for corner in t_corners],
            z=[corner[2] for corner in t_corners],
            opacity=0.8,
            color='red'
        ))
        fig.add_annotation(x=(t_x1 + t_x2) / 2, y=(t_y1 + t_y2) / 2,
                           text="End", showarrow=True, arrowhead=2, ax=0, ay=-40, font=dict(color='red'))

    # 绘制起点路径
    for (start, end) in s_path:
        # xs = [start[0], end[0]]
        # ys = [start[1], end[1]]
        # zs = [s_z, s_z]  # 使用起点的层高度
        s_x1, s_x2 = min(start[0],end[0]) - LINE_WIDTH_S / 2,max(start[0],end[0]) + LINE_WIDTH_S / 2
        s_y1, s_y2 = min(start[1],end[1]) - LINE_WIDTH_S / 2,max(start[1],end[1]) + LINE_WIDTH_S / 2
        z = s1[0][2]
        s_path_corners = [
            (s_x1, s_y1, z),
            (s_x2, s_y1, z),
            (s_x2, s_y2, z),
            (s_x1, s_y2, z)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in s_path_corners],
            y=[corner[1] for corner in s_path_corners],
            z=[corner[2] for corner in s_path_corners],
            opacity=0.8,
            color='blue'
        ))
        # fig.add_trace(go.Scatter3d(
        #     x=xs,
        #     y=ys,
        #     z=zs,
        #     mode='lines',
        #     line=dict(color='blue', width=4),
        #     name='s_path'
        # ))

    # 绘制终点路径
    # for (start, end) in t_path:
    #     xs = [start[0], end[0]]
    #     ys = [start[1], end[1]]
    #     zs = [t_z + 1, t_z + 1] if s[2] == t[2] else [t_z, t_z]
    #     fig.add_trace(go.Scatter3d(
    #         x=xs,
    #         y=ys,
    #         z=zs,
    #         mode='lines',
    #         line=dict(color='orange', width=4),
    #         name='t_path'
    #     ))
    for (start, end) in t_path:
        s_x1, s_x2 = min(start[0],end[0]) - LINE_WIDTH_S / 2,max(start[0],end[0]) + LINE_WIDTH_S / 2
        s_y1, s_y2 = min(start[1],end[1]) - LINE_WIDTH_S / 2,max(start[1],end[1]) + LINE_WIDTH_S / 2
        z = t1[0][2] + 1 if s1[0][2] == t1[0][2] else t1[0][2]
        s_path_corners = [
            (s_x1, s_y1, z),
            (s_x2, s_y1, z),
            (s_x2, s_y2, z),
            (s_x1, s_y2, z)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in s_path_corners],
            y=[corner[1] for corner in s_path_corners],
            z=[corner[2] for corner in s_path_corners],
            opacity=0.8,
            color='green'
        ))
    # 绘制通孔
    for (coord, layer) in res_via:
        x, y = coord
        z_start = layer_heights[0]
        z_end = layer_heights[0] + 1
        fig.add_trace(go.Scatter3d(
            x=[x, x],
            y=[y, y],
            z=[z_start, z_end],
            mode='lines',
            line=dict(color='black', width=4),
            name='Vias'
        ))
    # 绘制通孔
    for (coord, layer) in res_via:
        x, y = coord
        z_start = layer_heights[0]
        z_end = layer_heights[0] + 1

        s_x3, s_x4 = x - rect_center - rect_center / 2, x - rect_center + rect_center / 2
        s_y3, s_y4 = y - rect_center / 2, y + rect_center / 2

        s_x5, s_x6 = x + rect_center - rect_center / 2, x + rect_center + rect_center / 2
        s_y5, s_y6 = y - rect_center / 2, y + rect_center / 2

        s_x1, s_x2 = x - VIA_LENGTH / 2, x + VIA_LENGTH / 2
        s_y1, s_y2 = y - VIA_WIDTH / 2, y + VIA_WIDTH / 2

        via_path_corners = [
            (s_x3, s_y3, z_end),
            (s_x4, s_y3, z_end),
            (s_x4, s_y4, z_end),
            (s_x3, s_y4, z_end)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='magenta'
        ))
        via_path_corners = [
            (s_x3, s_y3, z_start),
            (s_x4, s_y3, z_start),
            (s_x4, s_y4, z_start),
            (s_x3, s_y4, z_start)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='magenta'
        ))

        via_path_corners = [
            (s_x5, s_y5, z_end),
            (s_x6, s_y5, z_end),
            (s_x6, s_y6, z_end),
            (s_x5, s_y6, z_end)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='magenta'
        ))
        via_path_corners = [
            (s_x5, s_y5, z_start),
            (s_x6, s_y5, z_start),
            (s_x6, s_y6, z_start),
            (s_x5, s_y6, z_start)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='magenta'
        ))

        via_path_corners = [
            (s_x1, s_y1, z_end),
            (s_x2, s_y1, z_end),
            (s_x2, s_y2, z_end),
            (s_x1, s_y2, z_end)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='rgba(0, 0, 0, 0.75)'
        ))
        via_path_corners = [
            (s_x1, s_y1, z_start),
            (s_x2, s_y1, z_start),
            (s_x2, s_y2, z_start),
            (s_x1, s_y2, z_start)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='rgba(0, 0, 0, 0.75)'
        ))

    # 通过设置比例来使得 z 轴看起来更低
    fig.update_layout(scene=dict(
        aspectmode='manual',  # 手动设置比例
        aspectratio=dict(x=1, y=1, z=0.4)  # z 轴的比例设置为 0.5，调整到合适的值
    ))
    # 设置坐标轴标签和范围
    fig.update_layout(scene=dict(
        xaxis_title='X',
        yaxis_title='Y',
        zaxis_title='Layer',
        zaxis=dict(
            range=[-1, max(layers) + 1],  # 调整 z 轴范围
            dtick=1  # 设置 z 轴刻度间隔为 1
        )
    ))

    # 添加标题
    fig.update_layout(title="3D Pattern Routing Visualization")

    # 显示图形
    fig.show()

def visualize_routing_3_layers(s1, t1, s_path, t_path, res_via,res_via1, obstacle_rects):
    """
    3D 可视化布线路径、通孔、障碍物、起点和终点的矩形框。

    参数:
    - s: 起始矩形，格式为 ((x1, y1), (x2, y2), z)
    - t: 目标矩形，格式为 ((x1, y1), (x2, y2), z)
    - s_path: 起点层路径列表，格式为 [((x1, y1), (x2, y2)), ...]
    - t_path: 终点层路径列表，格式为 [((x1, y1), (x2, y2)), ...]
    - res_via: 通孔列表，格式为 [( (x, y), layer ), ...]
    - obstacle_rects: 障碍物字典，格式为 {"layer_key": [((a,b),(c,d)), ...]}
    """
    # 创建图形
    fig = go.Figure()

    # 获取所有层数，包括起点和终点所在层
    layers = sorted(set(int(re.search(r'\d+', layer).group()) for layer in obstacle_rects.keys()))
    layers += [s1[0][2], t1[0][2]]  # 确保起点和终点层数包含在内
    layers = sorted(set(layers))  # 去重并排序

    layer_heights = {layer: layer for layer in layers}  # 定义层高度

    # 定义不同层的颜色
    color_map = {
        0: 'rgba(0, 0, 0, 1)',      # 第一层颜色：黑色
        1: 'rgba(50, 50, 50, 0.8)',   # 第二层颜色：深灰色
        # 可以添加更多层的颜色
        2: 'blue',
        3: 'purple'
    }

    # 绘制障碍物
    for layer, rects in obstacle_rects.items():
        layer_num = int(re.search(r'\d+', layer).group())  # 获取层数
        z = layer_heights[layer_num]  # 获取层高度
        for (x1, y1), (x2, y2) in rects:
            # 定义矩形的四个角
            corners = [
                (x1, y1, z),
                (x2, y1, z),
                (x2, y2, z),
                (x1, y2, z)
            ]
            # 创建多边形
            fig.add_trace(go.Mesh3d(
                x=[corner[0] for corner in corners],
                y=[corner[1] for corner in corners],
                z=[corner[2] for corner in corners],
                opacity=0.5,
                color=color_map.get(layer_num, 'grey')  # 使用层对应的颜色，默认为灰色
            ))
    for s in s1:
        # 绘制起点矩形框
        s_x1, s_y1 = s[0]
        s_x2, s_y2 = s[1]
        s_z = layer_heights[s[2]]
        s_corners = [
            (s_x1, s_y1, s_z),
            (s_x2, s_y1, s_z),
            (s_x2, s_y2, s_z),
            (s_x1, s_y2, s_z)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in s_corners],
            y=[corner[1] for corner in s_corners],
            z=[corner[2] for corner in s_corners],
            opacity=0.8,
            color='red'
        ))
        fig.add_annotation(x=(s_x1 + s_x2) / 2, y=(s_y1 + s_y2) / 2,
                           text="Start", showarrow=True, arrowhead=2, ax=0, ay=-40, font=dict(color='green'))
    for t in t1:
        # 绘制终点矩形框
        t_x1, t_y1 = t[0]
        t_x2, t_y2 = t[1]
        t_z = layer_heights[t[2]]
        t_corners = [
            (t_x1, t_y1, t_z),
            (t_x2, t_y1, t_z),
            (t_x2, t_y2, t_z),
            (t_x1, t_y2, t_z)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in t_corners],
            y=[corner[1] for corner in t_corners],
            z=[corner[2] for corner in t_corners],
            opacity=0.8,
            color='red'
        ))
        fig.add_annotation(x=(t_x1 + t_x2) / 2, y=(t_y1 + t_y2) / 2,
                           text="End", showarrow=True, arrowhead=2, ax=0, ay=-40, font=dict(color='red'))

    # 绘制起点路径
    for (start, end) in s_path:

        s_x1, s_x2 = min(start[0],end[0]) - LINE_WIDTH_S / 2,max(start[0],end[0]) + LINE_WIDTH_S / 2
        s_y1, s_y2 = min(start[1],end[1]) - LINE_WIDTH_S / 2,max(start[1],end[1]) + LINE_WIDTH_S / 2
        z = s1[0][2] + 1
        s_path_corners = [
            (s_x1, s_y1, z),
            (s_x2, s_y1, z),
            (s_x2, s_y2, z),
            (s_x1, s_y2, z)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in s_path_corners],
            y=[corner[1] for corner in s_path_corners],
            z=[corner[2] for corner in s_path_corners],
            opacity=0.8,
            color='blue'
        ))

    for (start, end) in t_path:
        s_x1, s_x2 = min(start[0],end[0]) - LINE_WIDTH_S / 2,max(start[0],end[0]) + LINE_WIDTH_S / 2
        s_y1, s_y2 = min(start[1],end[1]) - LINE_WIDTH_S / 2,max(start[1],end[1]) + LINE_WIDTH_S / 2
        z = t1[0][2] + 2
        s_path_corners = [
            (s_x1, s_y1, z),
            (s_x2, s_y1, z),
            (s_x2, s_y2, z),
            (s_x1, s_y2, z)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in s_path_corners],
            y=[corner[1] for corner in s_path_corners],
            z=[corner[2] for corner in s_path_corners],
            opacity=0.8,
            color='green'
        ))
    # 绘制通孔
    for (coord, layer) in res_via:
        x, y = coord
        z_start = layer_heights[0]
        z_end = layer_heights[0] + 1
        fig.add_trace(go.Scatter3d(
            x=[x, x],
            y=[y, y],
            z=[z_start, z_end],
            mode='lines',
            line=dict(color='black', width=4),
            name='Vias'
        ))
    # 绘制通孔
    for (coord, layer) in res_via:
        x, y = coord
        z_start = layer_heights[0]
        z_end = layer_heights[0] + 1

        s_x3, s_x4 = x - rect_center - rect_center / 2, x - rect_center + rect_center / 2
        s_y3, s_y4 = y - rect_center / 2, y + rect_center / 2

        s_x5, s_x6 = x + rect_center - rect_center / 2, x + rect_center + rect_center / 2
        s_y5, s_y6 = y - rect_center / 2, y + rect_center / 2

        s_x1, s_x2 = x - VIA_LENGTH / 2, x + VIA_LENGTH / 2
        s_y1, s_y2 = y - VIA_WIDTH / 2, y + VIA_WIDTH / 2

        via_path_corners = [
            (s_x3, s_y3, z_end),
            (s_x4, s_y3, z_end),
            (s_x4, s_y4, z_end),
            (s_x3, s_y4, z_end)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='magenta'
        ))
        via_path_corners = [
            (s_x3, s_y3, z_start),
            (s_x4, s_y3, z_start),
            (s_x4, s_y4, z_start),
            (s_x3, s_y4, z_start)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='magenta'
        ))

        via_path_corners = [
            (s_x5, s_y5, z_end),
            (s_x6, s_y5, z_end),
            (s_x6, s_y6, z_end),
            (s_x5, s_y6, z_end)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='magenta'
        ))
        via_path_corners = [
            (s_x5, s_y5, z_start),
            (s_x6, s_y5, z_start),
            (s_x6, s_y6, z_start),
            (s_x5, s_y6, z_start)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='magenta'
        ))

        via_path_corners = [
            (s_x1, s_y1, z_end),
            (s_x2, s_y1, z_end),
            (s_x2, s_y2, z_end),
            (s_x1, s_y2, z_end)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='rgba(0, 0, 0, 0.75)'
        ))
        via_path_corners = [
            (s_x1, s_y1, z_start),
            (s_x2, s_y1, z_start),
            (s_x2, s_y2, z_start),
            (s_x1, s_y2, z_start)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='rgba(0, 0, 0, 0.75)'
        ))

    # 绘制通孔
    for (coord, layer) in res_via1:
        x, y = coord
        z_start = layer_heights[0] + 1
        z_end = layer_heights[0] + 2
        fig.add_trace(go.Scatter3d(
            x=[x, x],
            y=[y, y],
            z=[z_start, z_end],
            mode='lines',
            line=dict(color='black', width=4),
            name='Vias'
        ))

        # 绘制通孔
    for (coord, layer) in res_via1:
        x, y = coord
        z_start = layer_heights[0] + 1
        z_end = layer_heights[0] + 2

        s_x3, s_x4 = x - rect_center1 - rect_center1 / 2, x - rect_center1 + rect_center1 / 2
        s_y3, s_y4 = y - rect_center1 / 2, y + rect_center1 / 2

        s_x5, s_x6 = x + rect_center1 - rect_center1 / 2, x + rect_center1 + rect_center1 / 2
        s_y5, s_y6 = y - rect_center1 / 2, y + rect_center1 / 2

        s_x1, s_x2 = x - VIA_LENGTH1 / 2, x + VIA_LENGTH1 / 2
        s_y1, s_y2 = y - VIA_WIDTH1 / 2, y + VIA_WIDTH1 / 2

        via_path_corners = [
            (s_x3, s_y3, z_end),
            (s_x4, s_y3, z_end),
            (s_x4, s_y4, z_end),
            (s_x3, s_y4, z_end)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='yellow'
        ))
        via_path_corners = [
            (s_x3, s_y3, z_start),
            (s_x4, s_y3, z_start),
            (s_x4, s_y4, z_start),
            (s_x3, s_y4, z_start)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='yellow'
        ))

        via_path_corners = [
            (s_x5, s_y5, z_end),
            (s_x6, s_y5, z_end),
            (s_x6, s_y6, z_end),
            (s_x5, s_y6, z_end)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='yellow'
        ))
        via_path_corners = [
            (s_x5, s_y5, z_start),
            (s_x6, s_y5, z_start),
            (s_x6, s_y6, z_start),
            (s_x5, s_y6, z_start)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='yellow'
        ))

        via_path_corners = [
            (s_x1, s_y1, z_end),
            (s_x2, s_y1, z_end),
            (s_x2, s_y2, z_end),
            (s_x1, s_y2, z_end)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='grey'
        ))
        via_path_corners = [
            (s_x1, s_y1, z_start),
            (s_x2, s_y1, z_start),
            (s_x2, s_y2, z_start),
            (s_x1, s_y2, z_start)
        ]
        fig.add_trace(go.Mesh3d(
            x=[corner[0] for corner in via_path_corners],
            y=[corner[1] for corner in via_path_corners],
            z=[corner[2] for corner in via_path_corners],
            opacity=0.8,
            color='grey'
        ))

    # 通过设置比例来使得 z 轴看起来更低
    fig.update_layout(scene=dict(
        aspectmode='manual',  # 手动设置比例
        aspectratio=dict(x=1, y=1, z=0.4)  # z 轴的比例设置为 0.5，调整到合适的值
    ))
    # 设置坐标轴标签和范围
    fig.update_layout(scene=dict(
        xaxis_title='X',
        yaxis_title='Y',
        zaxis_title='Layer',
        zaxis=dict(
            range=[-1, max(layers) + 1],  # 调整 z 轴范围
            dtick=1  # 设置 z 轴刻度间隔为 1
        )
    ))

    # 添加标题
    fig.update_layout(title="3D Pattern Routing Visualization")

    # 显示图形
    fig.show()
# ------------------------- 主执行部分 -------------------------
if __name__ == "__main__":
    res_via1 = []
    # 定义起点和终点坐标 (x, y, z)
    s = [((100, 120), (120, 180), 0)]
    t = [((380, 300), (400, 400), 0)]


    # 定义层,随机赋障碍物
    layers = ["0_layer"]
    layers = ["0_layer","1_layer"]
    # layers = ["0_layer","1_layer","2_layer"]
    # layers = ["0_layer","1_layer","2_layer","3_layer"]
    random_rectangles = generate_random_rectangles(
        num_rectangles=60,
        min_width=15,
        max_width=75,
        min_height=15,
        max_height=75,
        layers=layers,
        obstacles_to_avoid=s + t
    )
    # print(random_rectangles)
    # 记录开始时间
    # 打印起点、终点和障碍物信息
    print("====== 输入数据 ======")
    print(f"起点坐标 (s): {s}")
    print(f"终点坐标 (t): {t}")
    print("\n障碍物信息：")
    for layer, rectangles in random_rectangles.items():
        print(f"{layer} 层的障碍物数量为:{len(rectangles)}")
    print("======================\n")

    # 进行路径规划
    start_time = time.time()

    s_path, t_path, res_via = find_path(s, t, random_rectangles)
    if s_path == [] and t_path == [] and res_via == [] and\
            s[0][2] == t[0][2]:
        s_path, t_path, res_via1, res_via = find_path_cross_layers(s, t, random_rectangles)
        end_time = time.time()
        visualize_routing_3_layers(s, t, s_path, t_path, res_via, res_via1,random_rectangles)
    else:
        end_time = time.time()
        # 进行3D可视化
        visualize_routing(s, t, s_path, t_path, res_via, random_rectangles)
    # 记录结束时间
    print(f"代码执行时间: {end_time - start_time:.4f} 秒")
    print("起点路径:", s_path)
    print("终点路径:", t_path)
    print("通孔列表:", res_via)
    print("第二层的通孔列表:", res_via1)



