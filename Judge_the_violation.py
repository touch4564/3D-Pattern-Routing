# -*- coding: utf-8 -*-
"""

通孔扩展框和重叠判断

"""
from rtree import index
# ------------------------- 参数定义 -------------------------
# 定义布线和通孔的各种参数，包括线宽、线距、通孔尺寸等
VIA_LENGTH = 9        # 通孔长度
VIA_WIDTH = 3.8       # 通孔宽度

VIA_LENGTH1 = 7        # 通孔长度
VIA_WIDTH1 = 3       # 通孔宽度

# 起点层线宽与线距
LINE_WIDTH_S = 2
LINE_SPACING_S = 2

# 终点层线宽与线距
LINE_WIDTH_T = 2
LINE_SPACING_T = 2

eps = 0

# 过滤距离计算
FILTER_DIS_S = LINE_WIDTH_S / 2 + LINE_SPACING_S +eps
FILTER_DIS_T = LINE_WIDTH_T / 2 + LINE_SPACING_T +eps

# 版图缩放因子
SCALE_FACTOR = 100
# ------------------------- DRC违规判断函数 -------------------------
def is_overlap(rect1, rect2):
    """
    判断两个矩形是否重叠。

    参数:
    - rect1: ((x1_min, y1_min), (x1_max, y1_max))
    - rect2: ((x2_min, y2_min), (x2_max, y2_max))

    返回:
    - 重叠返回True，否则False
    """
    (x1_min, y1_min), (x1_max, y1_max) = rect1
    (x2_min, y2_min), (x2_max, y2_max) = rect2

    # 检查是否在x或y轴上不重叠
    if x1_max <= x2_min or x2_max <= x1_min:
        return False
    if y1_max <= y2_min or y2_max <= y1_min:
        return False
    return True


def build_obstacle_rtree(obstacles):
    """
    构建障碍物的 R 树索引。
    """
    rtree_idx = index.Index()
    for i, ((x1, y1), (x2, y2)) in enumerate(obstacles):
        rtree_idx.insert(i, (x1, y1, x2, y2))
    return rtree_idx

def judge_DRC_violations_with_rtree(rtree_idx, rectangle):
    """
    使用 R 树判断是否有重叠。
    """
    for rect in rectangle:
        x_min, y_min = rect[0]
        x_max, y_max = rect[1]
        # 查询与当前矩形可能重叠的障碍物
        possible_overlaps = list(rtree_idx.intersection((x_min, y_min, x_max, y_max)))
        if possible_overlaps:  # 如果有重叠障碍物，返回 True
            return True
    return False

def create_obstacle_hash(obstacle_list):
    """
    将障碍物列表转换为哈希表（字典），以便于快速查找和判断重叠。

    参数:
    - obstacle_list: 障碍物列表，每个障碍物由两个对角线坐标表示，格式为 [((x1, y1), (x2, y2)), ...]

    返回:
    - obstacle_hash: 字典，键为障碍物的坐标元组 (x1, y1, x2, y2)，值为障碍物的索引
    """
    obstacle_hash = {}
    for index, ob in enumerate(obstacle_list):
        # 假设obstacle_list中的元素为矩形，格式为 ((x1, y1), (x2, y2))
        rect_start, rect_end = ob  # 解包障碍物的起始和结束坐标
        # 提取坐标值
        x1, y1 = rect_start
        x2, y2 = rect_end
        # 使用矩形的左下角和右上角坐标作为键，将障碍物的索引作为值存储在字典中
        obstacle_hash[(x1, y1, x2, y2)] = index
    return obstacle_hash

def judge_DRC_violations(obstacle_hash, rectangle):
    """
    判断给定的矩形是否与哈希表中的任何障碍物重叠，从而判断是否违反设计规则检查（DRC）。

    参数:
    - obstacle_hash: 障碍物的哈希表，格式为 {(x1, y1, x2, y2): index, ...}
    - rectangle: 要检查的矩形列表，每个矩形由两个对角线坐标表示，格式为 [((x_min, y_min), (x_max, y_max)), ...]

    返回:
    - bool: 如果有任何重叠，返回True；否则，返回False
    """
    # 如果要检查的矩形列表为空，或者障碍物哈希表为空，则不可能有重叠
    if not rectangle or not obstacle_hash:
        return False
    else:
        # 遍历要检查的每一个矩形
        for rect in rectangle:
            x1_min, y1_min = rect[0]  # 当前矩形的左下角坐标
            x1_max, y1_max = rect[1]  # 当前矩形的右上角坐标
            # 遍历哈希表中的每一个障碍物矩形
            for key in obstacle_hash:
                ob_x1, ob_y1, ob_x2, ob_y2 = key  # 哈希表中障碍物的坐标
                # 判断当前矩形是否与障碍物矩形重叠
                # 如果两个矩形在x轴或y轴上不重叠，则不会有重叠
                if not ((x1_max <= ob_x1 or ob_x2 <= x1_min) or (y1_max <= ob_y1 or ob_y2 <= y1_min)):
                    # 如果矩形与哈希表中的任何一个障碍物有重叠部分，返回True
                    return True
    # 如果所有矩形都没有与任何障碍物重叠，返回False
    return False

# ------------------------- 通孔和布线矩形框相关函数 -------------------------
def path_ob_discriminatory(segments):
    """
    修改路径以避开障碍物。

    参数:
    - segments: 路径线段列表

    返回:
    - 修改后的路径线段列表
    """
    modified_segments = []
    for segment in segments:
        (x1, y1), (x2, y2) = segment
        # 确保起点和终点的顺序
        start = (min(x1, x2), min(y1, y2))
        end = (max(x1, x2), max(y1, y2))

        if start[1] != end[1]:
            new_start = (start[0] - FILTER_DIS_T, start[1]- FILTER_DIS_T )
            new_end = (end[0] + FILTER_DIS_T, end[1] + FILTER_DIS_T)
            modified_segments.append((new_start, new_end))
        elif start[0] != end[0]:
            new_start = (start[0] - FILTER_DIS_T, start[1] - FILTER_DIS_T)
            new_end = (end[0] + FILTER_DIS_T , end[1] + FILTER_DIS_T)
            modified_segments.append((new_start, new_end))
    return modified_segments

def get_via_rect_s(node):
    """
    获取起点层通孔的矩形范围。

    参数:
    - node: (x, y) 坐标

    返回:
    - 矩形列表，格式为[((x_min, y_min), (x_max, y_max))]
    """
    x, y = node
    return [((x - VIA_LENGTH / 2 - LINE_SPACING_S  , y - VIA_WIDTH / 2 - LINE_SPACING_S ),
             (x + VIA_LENGTH / 2 + LINE_SPACING_S , y + VIA_WIDTH / 2 + LINE_SPACING_S ))]

def get_via_rect_t(node):
    """
    获取终点层通孔的矩形范围。

    参数:
    - node: (x, y) 坐标

    返回:
    - 矩形列表，格式为[((x_min, y_min), (x_max, y_max))]
    """
    x, y = node
    return [((x - VIA_LENGTH / 2 - LINE_SPACING_T , y - VIA_WIDTH / 2 - LINE_SPACING_T),
             (x + VIA_LENGTH / 2 + LINE_SPACING_T  , y + VIA_WIDTH / 2 + LINE_SPACING_T))]


def get_via_rect_s_v(node):
    """
    获取起点层通孔的矩形范围。

    参数:
    - node: (x, y) 坐标

    返回:
    - 矩形列表，格式为[((x_min, y_min), (x_max, y_max))]
    """
    x, y = node
    return [((x - VIA_WIDTH / 2 - LINE_SPACING_S  , y - VIA_LENGTH / 2 - LINE_SPACING_S ),
             (x + VIA_WIDTH / 2 + LINE_SPACING_S , y + VIA_LENGTH / 2 + LINE_SPACING_S ))]

def get_via_rect_t_v(node):
    """
    获取终点层通孔的矩形范围。

    参数:
    - node: (x, y) 坐标

    返回:
    - 矩形列表，格式为[((x_min, y_min), (x_max, y_max))]
    """
    x, y = node
    return [((x - VIA_WIDTH / 2 - LINE_SPACING_T , y - VIA_LENGTH / 2 - LINE_SPACING_T),
             (x + VIA_WIDTH / 2 + LINE_SPACING_T  , y + VIA_LENGTH / 2 + LINE_SPACING_T))]

if __name__ == "__main__":
    # 定义障碍物
    obstacles = [
        ((1, 1), (4, 4)),  # 障碍物 1
        ((5, 5), (7, 7)),  # 障碍物 2
        ((6, 1), (9, 3))  # 障碍物 3
    ]

    # 构建 R 树
    rtree_idx = build_obstacle_rtree(obstacles)

    # 定义要检测的矩形
    rectangle = [
        ((3, 3), (6, 6)),  # 要检测的矩形
        ((8, 8), (10, 10)) # 第二个矩形
]

    # 调用检测函数
    result = judge_DRC_violations_with_rtree(rtree_idx, rectangle)
    print(result)  # 输出：True（因为第一个矩形与障碍物 1 重叠）
