# -*- coding: utf-8 -*-
"""


起点引脚区域和终点引脚区域在同层或不同层的模式布线判别函数


"""
from pattern_routing.Judge_the_violation import path_ob_discriminatory, get_via_rect_s, get_via_rect_t, judge_DRC_violations,\
get_via_rect_s_v, get_via_rect_t_v
# ------------------------- 参数定义 -------------------------
# 定义布线和通孔的各种参数，包括线宽、线距、通孔尺寸等
VIA_LENGTH = 9        # 通孔长度
VIA_WIDTH = 3.8       # 通孔宽度

# 起点层线宽与线距
LINE_WIDTH_S = 2
LINE_SPACING_S = 2

# 终点层线宽与线距
LINE_WIDTH_T = 2
LINE_SPACING_T = 2
# ------------------------- 不同层路径检查函数 -------------------------
def check_and_return_path_x(start_x, start_y, inter_x, end_x, end_y, rect_s, rect_t, rect_via):
    """
    检查路径段和通孔是否与障碍物重叠，并返回路径和通孔（水平优先）。

    参数:
    - start_x, start_y: 起点坐标
    - inter_x: 中间拐点的 x 坐标
    - end_x, end_y: 终点坐标
    - rect_s: 起始层障碍物
    - rect_t: 终止层障碍物
    - rect_via: 所有层通孔障碍物

    返回:
    - s_path: 起点层路径
    - t_path: 终点层路径
    - vias: 通孔列表
    """
    node0 = (start_x, start_y)
    node1 = (inter_x, start_y)
    node2 = (inter_x, end_y)
    node3 = (end_x, end_y)

    # 定义路径段
    seg = [] if node0 == node1 else [(node0, node1)]
    seg1 = [(node1, node2)]
    seg2 = [] if node2 == node3 else [(node2, node3)]

    # 修改路径以避开障碍物
    seg_rect_s = path_ob_discriminatory(seg)
    seg_rect_s1 = path_ob_discriminatory(seg1)
    seg_rect_s2 = path_ob_discriminatory(seg2)

    # 定义通孔位置
    via0 = node0
    via = node1
    via1 = node2
    via2 = node3

    # 获取通孔矩形
    via_s0 = get_via_rect_s(via0)
    via_s = get_via_rect_s(via)
    via_s1 = get_via_rect_s(via1)
    via_s2 = get_via_rect_t(via2)

    # 检查各路径段和通孔是否与障碍物重叠
    condition1 = (
            abs(end_y - start_y) >= VIA_LENGTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2 and
            not judge_DRC_violations(rect_s, seg_rect_s) and
            not judge_DRC_violations(rect_s, seg_rect_s1) and
            not judge_DRC_violations(rect_t, seg_rect_s2) and
            not judge_DRC_violations(rect_via, via_s1)
    )

    condition2 = (
            abs(end_x - inter_x) >= VIA_WIDTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2 and
            not judge_DRC_violations(rect_s, seg_rect_s) and
            not judge_DRC_violations(rect_s, seg_rect_s1) and
            not judge_DRC_violations(rect_s, seg_rect_s2) and
            not judge_DRC_violations(rect_via, via_s2)
    )

    condition3 = (
            abs(end_y - start_y) >= VIA_LENGTH / 2 + LINE_SPACING_S + LINE_WIDTH_T / 2 and
            not judge_DRC_violations(rect_s, seg_rect_s) and
            not judge_DRC_violations(rect_t, seg_rect_s1) and
            not judge_DRC_violations(rect_t, seg_rect_s2) and
            not judge_DRC_violations(rect_via, via_s)
    )

    condition4 = (
            abs(inter_x - start_x) >= VIA_WIDTH / 2 + LINE_SPACING_T + LINE_WIDTH_T / 2 and
            not judge_DRC_violations(rect_t, seg_rect_s) and
            not judge_DRC_violations(rect_t, seg_rect_s1) and
            not judge_DRC_violations(rect_t, seg_rect_s2) and
            not judge_DRC_violations(rect_via, via_s0)
    )

    # 根据条件返回相应的路径和通孔
    if condition1:
        s_path = seg + seg1
        t_path = seg2
        return s_path, t_path, [(via1, 1)]
    elif condition2:
        s_path = seg + seg1 + seg2
        t_path = []
        return s_path, t_path, [(via2, 1)]
    elif condition3:
        s_path = seg
        t_path = seg1 + seg2
        return s_path, t_path, [(via, 1)]
    elif condition4:
        s_path = []
        t_path = seg + seg1 + seg2
        return s_path, t_path, [(via0, 1)]

    return None, None, None

def check_and_return_path_y(start_x, start_y, inter_y, end_x, end_y, rect_s, rect_t, rect_via):
    """
    检查路径段和通孔是否与障碍物重叠，并返回路径和通孔（垂直优先）。

    参数:
    - start_x, start_y: 起点坐标
    - inter_y: 中间拐点的 y 坐标
    - end_x, end_y: 终点坐标
    - rect_s: 起始层障碍物
    - rect_t: 终止层障碍物
    - rect_via: 所有层通孔障碍物

    返回:
    - s_path: 起点层路径
    - t_path: 终点层路径
    - vias: 通孔列表
    """
    node0 = (start_x, start_y)
    node1 = (start_x, inter_y)
    node2 = (end_x, inter_y)
    node3 = (end_x, end_y)

    # 定义路径段
    seg = [] if node0 == node1 else [(node0, node1)]
    seg1 = [(node1, node2)]
    seg2 = [] if node2 == node3 else [(node2, node3)]

    # 修改路径以避开障碍物
    seg_rect_s = path_ob_discriminatory(seg)
    seg_rect_s1 = path_ob_discriminatory(seg1)
    seg_rect_s2 = path_ob_discriminatory(seg2)

    # 定义通孔位置
    via0 = node0
    via = node1
    via1 = node2
    via2 = node3

    # 获取通孔矩形
    via_s0 = get_via_rect_s(via0)
    via_s = get_via_rect_s(via)
    via_s1 = get_via_rect_s(via1)
    via_s2 = get_via_rect_t(via2)

    # 检查各路径段和通孔是否与障碍物重叠
    condition1 = (
            abs(end_x - start_x) >= VIA_WIDTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2 and
            not judge_DRC_violations(rect_s, seg_rect_s) and
            not judge_DRC_violations(rect_s, seg_rect_s1) and
            not judge_DRC_violations(rect_t, seg_rect_s2) and
            not judge_DRC_violations(rect_via, via_s1)
    )

    condition2 = (
            abs(end_y - inter_y) >= VIA_LENGTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2 and
            not judge_DRC_violations(rect_s, seg_rect_s) and
            not judge_DRC_violations(rect_s, seg_rect_s1) and
            not judge_DRC_violations(rect_s, seg_rect_s2) and
            not judge_DRC_violations(rect_via, via_s2)
    )

    condition3 = (
            abs(end_x - start_x) >= VIA_WIDTH / 2 + LINE_SPACING_S + LINE_WIDTH_T / 2 and
            not judge_DRC_violations(rect_s, seg_rect_s) and
            not judge_DRC_violations(rect_t, seg_rect_s1) and
            not judge_DRC_violations(rect_t, seg_rect_s2) and
            not judge_DRC_violations(rect_via, via_s)
    )

    condition4 = (
            abs(inter_y - start_y) >= VIA_LENGTH / 2 + LINE_SPACING_T + LINE_WIDTH_T / 2 and
            not judge_DRC_violations(rect_t, seg_rect_s) and
            not judge_DRC_violations(rect_t, seg_rect_s1) and
            not judge_DRC_violations(rect_t, seg_rect_s2) and
            not judge_DRC_violations(rect_via, via_s0)
    )

    # 根据条件返回相应的路径和通孔
    if condition1:
        s_path = seg + seg1
        t_path = seg2
        return s_path, t_path, [(via1, 1)]
    elif condition2:
        s_path = seg + seg1 + seg2
        t_path = []
        return s_path, t_path, [(via2, 1)]
    elif condition3:
        s_path = seg
        t_path = seg1 + seg2
        return s_path, t_path, [(via, 1)]
    elif condition4:
        s_path = []
        t_path = seg + seg1 + seg2
        return s_path, t_path, [(via0, 1)]

    return None, None, None

# ------------------------- 同层路径检查函数 -------------------------
def check_same_layer_x(start_x, start_y, inter_x, end_x, end_y, rect):
    """
    检查在同一层上沿X方向进行L型路径布线是否与障碍物冲突。

    参数:
        start_x (int): 起点的X坐标
        start_y (int): 起点的Y坐标
        inter_x (int): 拐点的X坐标
        end_x (int): 终点的X坐标
        end_y (int): 终点的Y坐标
        rect (dict): 障碍物的哈希表

    返回:
        tuple: 如果路径可行，返回路径列表和空列表；否则返回 (None, None, None)
    """
    # 定义路径上的节点
    node0 = (start_x, start_y)
    node1 = (inter_x, start_y)
    node2 = (inter_x, end_y)
    node3 = (end_x, end_y)

    # 定义路径段，避免起点与拐点相同的情况
    seg = [] if node0 == node1 else [(node0, node1)]
    seg1 = [(node1, node2)]
    seg2 = [] if node2 == node3 else [(node2, node3)]

    # 修改路径以避开障碍物
    seg_rect_s = path_ob_discriminatory(seg)
    seg_rect_s1 = path_ob_discriminatory(seg1)
    seg_rect_s2 = path_ob_discriminatory(seg2)

    # 检查各路径段是否与障碍物冲突
    condition = (
            not judge_DRC_violations(rect, seg_rect_s) and
            not judge_DRC_violations(rect, seg_rect_s1) and
            not judge_DRC_violations(rect, seg_rect_s2)
    )

    # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
    if condition:
        s_path = seg + seg1 + seg2
        return s_path, [], []

    # 如果存在冲突，返回空结果
    return None, None, None

def check_same_layer_y(start_x, start_y, inter_y, end_x, end_y, rect):
    """
    检查在同一层上沿Y方向进行L型路径布线是否与障碍物冲突。

    参数:
        start_x (int): 起点的X坐标
        start_y (int): 起点的Y坐标
        inter_y (int): 拐点的Y坐标
        end_x (int): 终点的X坐标
        end_y (int): 终点的Y坐标
        rect (dict): 障碍物的哈希表

    返回:
        tuple: 如果路径可行，返回路径列表和空列表；否则返回 (None, None, None)
    """
    # 定义路径上的节点
    node0 = (start_x, start_y)
    node1 = (start_x, inter_y)
    node2 = (end_x, inter_y)
    node3 = (end_x, end_y)

    # 定义路径段，避免起点与拐点相同的情况
    seg = [] if node0 == node1 else [(node0, node1)]
    seg1 = [(node1, node2)]
    seg2 = [] if node2 == node3 else [(node2, node3)]

    # 修改路径以避开障碍物
    seg_rect_s = path_ob_discriminatory(seg)
    seg_rect_s1 = path_ob_discriminatory(seg1)
    seg_rect_s2 = path_ob_discriminatory(seg2)

    # 检查各路径段是否与障碍物冲突
    condition = (
            not judge_DRC_violations(rect, seg_rect_s) and
            not judge_DRC_violations(rect, seg_rect_s1) and
            not judge_DRC_violations(rect, seg_rect_s2)
    )

    # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
    if condition:
        s_path = seg + seg1 + seg2
        return s_path, [], []

    # 如果存在冲突，返回空结果
    return None, None, None

# ------------------------- 同层路径带通孔检查函数 -------------------------
def check_same_layer_x_via(start_x, start_y, inter_x, end_x, end_y, rect, rect1):
    """
    检查在同一层上沿X方向进行Z型路径布线是否与障碍物冲突。

    参数:
        start_x (int): 起点的X坐标
        start_y (int): 起点的Y坐标
        inter_x (int): 拐点的X坐标
        end_x (int): 终点的X坐标
        end_y (int): 终点的Y坐标
        rect (dict): 障碍物的哈希表
        rect1 (dict): 向上层的障碍物的哈希表

    返回:
        tuple: 如果路径可行，返回路径列表和空列表；否则返回 (None, None, None)
    """
    # 定义路径上的节点
    node0 = (start_x, start_y)
    node1 = (inter_x, start_y)
    node2 = (inter_x, end_y)
    node3 = (end_x, end_y)

    # 定义路径段，避免起点与拐点相同的情况
    seg = [] if node0 == node1 else [(node0, node1)]
    seg1 = [(node1, node2)]
    seg2 = [] if node2 == node3 else [(node2, node3)]

    # 修改路径以避开障碍物
    seg_rect_s = path_ob_discriminatory(seg)
    seg_rect_s1 = path_ob_discriminatory(seg1)
    seg_rect_s2 = path_ob_discriminatory(seg2)


    via = node1
    via1 = node2
    viaS = node0
    viaT = node3

    # 获取通孔矩形
    via_s = get_via_rect_s(via)
    via_s1 = get_via_rect_s(via1)
    via_S = get_via_rect_s(viaS)
    via_T = get_via_rect_s(viaT)

    if abs(start_y - end_y) >= VIA_WIDTH + LINE_SPACING_S:
        # 检查各路径段是否与障碍物冲突
        condition = (
                not judge_DRC_violations(rect, seg_rect_s) and
                not judge_DRC_violations(rect1, seg_rect_s1) and
                not judge_DRC_violations(rect, seg_rect_s2) and
                not judge_DRC_violations(rect, via_s) and
                not judge_DRC_violations(rect1, via_s) and
                not judge_DRC_violations(rect, via_s1) and
                not judge_DRC_violations(rect1, via_s1)
        )

        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition:
            s_path = seg + seg2
            t_path = seg1
            return s_path, t_path, [(via, 1), (via1, 1)]
    # 检查各路径段是否与障碍物冲突
    condition = (
            not judge_DRC_violations(rect, seg_rect_s) and
            not judge_DRC_violations(rect, seg_rect_s1) and
            not judge_DRC_violations(rect, seg_rect_s2)
    )

    # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
    if condition:
        s_path = seg + seg2 +seg1
        t_path = []
        return s_path, t_path, []
    if abs(inter_x - start_x) >= VIA_LENGTH/2 + LINE_SPACING_S + LINE_WIDTH_S /2 and \
            abs(start_y - end_y) >= VIA_WIDTH/2 + LINE_SPACING_S + LINE_WIDTH_S /2 :
        condition = (
                not judge_DRC_violations(rect1, seg_rect_s) and
                not judge_DRC_violations(rect1, seg_rect_s1) and
                not judge_DRC_violations(rect, seg_rect_s2) and
                not judge_DRC_violations(rect, via_S) and
                not judge_DRC_violations(rect1, via_S) and
                not judge_DRC_violations(rect, via_s1) and
                not judge_DRC_violations(rect1, via_s1)
        )

        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition:
            s_path = seg2
            t_path = seg + seg1
            return s_path, t_path, [(viaS, 1), (via1, 1)]
    if abs(inter_x - end_x) >= VIA_LENGTH/2 + LINE_SPACING_S + LINE_WIDTH_S /2 and \
            abs(start_y - end_y) >= VIA_WIDTH/2 + LINE_SPACING_S + LINE_WIDTH_S /2 :
        condition = (
                not judge_DRC_violations(rect, seg_rect_s) and
                not judge_DRC_violations(rect1, seg_rect_s1) and
                not judge_DRC_violations(rect1, seg_rect_s2) and
                not judge_DRC_violations(rect, via_T) and
                not judge_DRC_violations(rect1, via_T) and
                not judge_DRC_violations(rect, via_s) and
                not judge_DRC_violations(rect1, via_s)
        )

        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition:
            s_path = seg
            t_path = seg2 + seg1
            return s_path, t_path, [(viaT, 1), (via, 1)]

    # 如果存在冲突，返回空结果
    return None, None, None

def check_same_layer_y_via(start_x, start_y, inter_y, end_x, end_y, rect, rect1):
    """
    检查在同一层上沿Y方向进行Z型路径布线是否与障碍物冲突。

    参数:
        start_x (int): 起点的X坐标
        start_y (int): 起点的Y坐标
        inter_y (int): 拐点的Y坐标
        end_x (int): 终点的X坐标
        end_y (int): 终点的Y坐标
        rect (dict): 障碍物的哈希表
        rect1 (dict): 向上层的障碍物的哈希表

    返回:
        tuple: 如果路径可行，返回路径列表和空列表；否则返回 (None, None, None)
    """
    # 定义路径上的节点
    node0 = (start_x, start_y)
    node1 = (start_x, inter_y)
    node2 = (end_x, inter_y)
    node3 = (end_x, end_y)

    # 定义路径段，避免起点与拐点相同的情况
    seg = [] if node0 == node1 else [(node0, node1)]
    seg1 = [(node1, node2)]
    seg2 = [] if node2 == node3 else [(node2, node3)]

    # 修改路径以避开障碍物
    seg_rect_s = path_ob_discriminatory(seg)
    seg_rect_s1 = path_ob_discriminatory(seg1)
    seg_rect_s2 = path_ob_discriminatory(seg2)

    via = node1
    via1 = node2
    viaS = node0
    viaT = node3

    # 获取通孔矩形
    via_s = get_via_rect_s(via)
    via_s1 = get_via_rect_s(via1)
    via_S = get_via_rect_s(viaS)
    via_T = get_via_rect_s(viaT)

    if abs(start_x - end_x) >= VIA_LENGTH + LINE_SPACING_S:
        # 检查各路径段是否与障碍物冲突
        condition = (
                not judge_DRC_violations(rect, seg_rect_s) and
                not judge_DRC_violations(rect1, seg_rect_s1) and
                not judge_DRC_violations(rect, seg_rect_s2) and
                not judge_DRC_violations(rect, via_s) and
                not judge_DRC_violations(rect1, via_s) and
                not judge_DRC_violations(rect, via_s1) and
                not judge_DRC_violations(rect1, via_s1)
        )

        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition:
            s_path = seg + seg2
            t_path = seg1
            return s_path, t_path, [(via, 1), (via1, 1)]
    # 检查各路径段是否与障碍物冲突
    condition = (
            not judge_DRC_violations(rect, seg_rect_s) and
            not judge_DRC_violations(rect, seg_rect_s1) and
            not judge_DRC_violations(rect, seg_rect_s2)
    )
    # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
    if condition:
        s_path = seg + seg2 + seg1
        t_path = []
        return s_path, t_path, []
    if abs(inter_y - start_y) >= VIA_WIDTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2 and \
            abs(start_x - end_x) >= VIA_LENGTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2:
        condition = (
                not judge_DRC_violations(rect1, seg_rect_s) and
                not judge_DRC_violations(rect1, seg_rect_s1) and
                not judge_DRC_violations(rect, seg_rect_s2) and
                not judge_DRC_violations(rect, via_S) and
                not judge_DRC_violations(rect1, via_S) and
                not judge_DRC_violations(rect, via_s1) and
                not judge_DRC_violations(rect1, via_s1)
        )

        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition:
            s_path = seg2
            t_path = seg + seg1
            return s_path, t_path, [(viaS, 1), (via1, 1)]
    if abs(inter_y - end_y) >= VIA_WIDTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2 and \
            abs(start_x - end_x) >= VIA_LENGTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2:
        condition = (
                not judge_DRC_violations(rect, seg_rect_s) and
                not judge_DRC_violations(rect1, seg_rect_s1) and
                not judge_DRC_violations(rect1, seg_rect_s2) and
                not judge_DRC_violations(rect, via_T) and
                not judge_DRC_violations(rect1, via_T) and
                not judge_DRC_violations(rect, via_s) and
                not judge_DRC_violations(rect1, via_s)
        )

        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition:
            s_path = seg
            t_path = seg2 + seg1
            return s_path, t_path, [(viaT, 1), (via, 1)]
    # 如果存在冲突，返回空结果
    return None, None, None

# ------------------------- 同层或者不同层三转向和Z型模式路径检查函数 -------------------------
def check_path_all(start_x, start_y, x, y, end_x, end_y, rect, rect1, dict_node):
    """

    参数:
        start_x (int): 起点的X坐标
        start_y (int): 起点的Y坐标
        y (int): 拐点的Y坐标
        x (int): 拐点的X坐标
        end_x (int): 终点的X坐标
        end_y (int): 终点的Y坐标
        rect (dict): 障碍物的哈希表
        rect1 (dict): 向上层的障碍物的哈希表

    返回:
        tuple: 如果路径可行，返回路径列表和空列表；否则返回 (None, None, None)
    """
    # 定义路径上的节点
    p = (x, y)
    node0 = (start_x, start_y)
    node1 = (start_x, y)
    node2 = (end_x, y)
    node3 = (end_x, end_y)
    node4 = (x, start_y)
    node5 = (x, end_y)

    # 定义路径段，避免起点与拐点相同的情况
    seg_04 = [] if node0 == node4 else [(node0, node4)]
    seg_P4 = [] if p == node4 else [(node4, p)]

    seg_01 = [] if node0 == node1 else [(node0, node1)]
    seg_P1 = [] if p == node1 else [(node1, p)]

    seg_53 = [] if node5 == node3 else [(node5, node3)]
    seg_P5 = [] if p == node5 else [(p, node5)]

    seg_23 = [] if node2 == node3 else [(node2, node3)]
    seg_P2 = [] if p == node2 else [(p, node2)]

    # 修改路径以避开障碍物
    seg_rect_s04 = path_ob_discriminatory(seg_04)
    seg_rect_sP4 = path_ob_discriminatory(seg_P4)
    via1 = node4

    seg_rect_s01 = path_ob_discriminatory(seg_01)
    seg_rect_sP1 = path_ob_discriminatory(seg_P1)
    via2 = node1


    seg_rect_s53 = path_ob_discriminatory(seg_53)
    seg_rect_sP5 = path_ob_discriminatory(seg_P5)
    via3 = node5

    seg_rect_s23 = path_ob_discriminatory(seg_23)
    seg_rect_sP2 = path_ob_discriminatory(seg_P2)
    via4 = node2

    # 获取通孔矩形
    via_s1 = get_via_rect_s(via1)
    via_s2 = get_via_rect_s(via2)
    via_s3 = get_via_rect_s(via3)
    via_s4 = get_via_rect_s(via4)
    via_p = get_via_rect_s(p)
    # Z型模式布线
    if (x, 0.1) not in dict_node:  # 检查节点 (x, 0.1) 是否已经存在于字典中
        # 检查另一种路径情况
        condition1 = (
                not judge_DRC_violations(rect, seg_rect_s04) and
                not judge_DRC_violations(rect, seg_rect_sP4) and
                not judge_DRC_violations(rect, seg_rect_s53) and
                not judge_DRC_violations(rect, seg_rect_sP5)
        )
        dict_node[(x, 0.1)] = 1  # 记录节点 (x, 0.1)

        # 如果没有冲突，返回组合后的路径和空的通孔列表
        if condition1:
            s_path = seg_04 + seg_53 + seg_P4 + seg_P5  # 源路径
            t_path = []  # 目标路径为空
            return s_path, t_path, [], dict_node  # 返回路径和通孔列表

        # 检查起始和结束的 y 坐标差是否大于或等于通孔宽度加线间距
        if abs(start_y - end_y) >= VIA_WIDTH + LINE_SPACING_S:
            # 检查各路径段是否与障碍物冲突
            condition1 = (
                    not judge_DRC_violations(rect, seg_rect_s04) and
                    not judge_DRC_violations(rect1, seg_rect_sP4) and
                    not judge_DRC_violations(rect, seg_rect_s53) and
                    not judge_DRC_violations(rect1, seg_rect_sP5) and
                    not judge_DRC_violations(rect, via_s1) and
                    not judge_DRC_violations(rect1, via_s1) and
                    not judge_DRC_violations(rect, via_s3) and
                    not judge_DRC_violations(rect1, via_s3)
            )

            # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
            if condition1:
                s_path = seg_04 + seg_53  # 源路径
                t_path = seg_P4 + seg_P5  # 目标路径
                return s_path, t_path, [(via1, 1), (via3, 1)], dict_node  # 返回路径和通孔列表


    # 检查 (y, -0.1) 节点
    if (y, -0.1) not in dict_node:  # 检查节点 (y, -0.1) 是否已经存在于字典中
        # 检查另一种路径情况
        condition2 = (
                not judge_DRC_violations(rect, seg_rect_s01) and
                not judge_DRC_violations(rect, seg_rect_sP1) and
                not judge_DRC_violations(rect, seg_rect_s23) and
                not judge_DRC_violations(rect, seg_rect_sP2)
        )
        dict_node[(y, -0.1)] = 1  # 记录节点 (y, -0.1)

        # 如果没有冲突，返回组合后的路径和空的通孔列表
        if condition2:
            s_path = seg_01 + seg_23 + seg_P1 + seg_P2  # 源路径
            t_path = []  # 目标路径为空
            return s_path, t_path, [], dict_node  # 返回路径和通孔列表

        # 检查起始和结束的 x 坐标差是否大于或等于通孔长度加线间距
        if abs(start_x - end_x) >= VIA_LENGTH + LINE_SPACING_S:
            # 检查各路径段是否与障碍物冲突
            condition2 = (
                    not judge_DRC_violations(rect, seg_rect_s01) and
                    not judge_DRC_violations(rect1, seg_rect_sP1) and
                    not judge_DRC_violations(rect, seg_rect_s23) and
                    not judge_DRC_violations(rect1, seg_rect_sP2) and
                    not judge_DRC_violations(rect, via_s2) and
                    not judge_DRC_violations(rect1, via_s4) and
                    not judge_DRC_violations(rect1, via_s2) and
                    not judge_DRC_violations(rect, via_s4)
            )

            # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
            if condition2:
                s_path = seg_01 + seg_23  # 源路径
                t_path = seg_P1 + seg_P2  # 目标路径
                return s_path, t_path, [(via2, 1), (via4, 1)], dict_node  # 返回路径和通孔列表

        # 检查各路径段是否与障碍物冲突
    condition5 = (
            not judge_DRC_violations(rect, seg_rect_s04) and
            not judge_DRC_violations(rect, seg_rect_sP4) and
            not judge_DRC_violations(rect, seg_rect_s23) and
            not judge_DRC_violations(rect, seg_rect_sP2)
    )
    # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
    if condition5:
        s_path = seg_04 + seg_23 + seg_P4 + seg_P2
        t_path = []
        return s_path, t_path, [], dict_node

        # 检查各路径段是否与障碍物冲突
    condition6 = (
            not judge_DRC_violations(rect, seg_rect_s01) and
            not judge_DRC_violations(rect, seg_rect_sP1) and
            not judge_DRC_violations(rect, seg_rect_s53) and
            not judge_DRC_violations(rect, seg_rect_sP5)
    )
    # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
    if condition6:
        s_path = seg_01 + seg_53 + seg_P1 + seg_P5
        t_path = []
        return s_path, t_path, [], dict_node


    # 三转向型模式布线
    if abs(y - start_y) > VIA_WIDTH/2 + LINE_SPACING_S + LINE_WIDTH_S/2 and abs(x - end_x) > VIA_LENGTH/2 + LINE_SPACING_S + LINE_WIDTH_S/2:
        # 检查各路径段是否与障碍物冲突
        condition3 = (
                not judge_DRC_violations(rect, seg_rect_s04) and
                not judge_DRC_violations(rect1, seg_rect_sP4) and
                not judge_DRC_violations(rect, seg_rect_s23) and
                not judge_DRC_violations(rect1, seg_rect_sP2)and not judge_DRC_violations(rect, via_s1) and
                not judge_DRC_violations(rect1, via_s4) and
                not judge_DRC_violations(rect1, via_s1) and
                not judge_DRC_violations(rect, via_s4)
        )
        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition3:
            s_path = seg_04 + seg_23
            t_path = seg_P4 + seg_P2
            return s_path, t_path, [(via1, 1), (via4, 1)], dict_node

    if abs(y - start_y) > VIA_WIDTH + LINE_SPACING_S and abs(x - end_x) > VIA_LENGTH/2 + LINE_SPACING_S + LINE_WIDTH_S/2:
        # 检查各路径段是否与障碍物冲突
        condition3 = (
                not judge_DRC_violations(rect, seg_rect_s04) and
                not judge_DRC_violations(rect1, seg_rect_sP4) and
                not judge_DRC_violations(rect, seg_rect_s23) and
                not judge_DRC_violations(rect, seg_rect_sP2)and not judge_DRC_violations(rect, via_s1) and
                not judge_DRC_violations(rect1, via_p) and
                not judge_DRC_violations(rect1, via_s1) and
                not judge_DRC_violations(rect, via_p)
        )
        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition3:
            s_path = seg_04 + seg_23 + seg_P2
            t_path = seg_P4
            return s_path, t_path, [(via1, 1), (p, 1)], dict_node
    if abs(y - start_y) > VIA_WIDTH/2 + LINE_SPACING_S + LINE_WIDTH_S/2 and abs(
            x - end_x) > VIA_LENGTH + LINE_SPACING_S:
        # 检查各路径段是否与障碍物冲突
        condition3 = (
                not judge_DRC_violations(rect, seg_rect_s04) and
                not judge_DRC_violations(rect, seg_rect_sP4) and
                not judge_DRC_violations(rect, seg_rect_s23) and
                not judge_DRC_violations(rect1, seg_rect_sP2) and not judge_DRC_violations(rect, via_s4) and
                not judge_DRC_violations(rect1, via_p) and
                not judge_DRC_violations(rect1, via_s4) and
                not judge_DRC_violations(rect, via_p)
        )
        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition3:
            s_path = seg_04 + seg_23 + seg_P4
            t_path = seg_P2
            return s_path, t_path, [(via4, 1), (p, 1)], dict_node
    if abs(y - end_y) > VIA_WIDTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2 and abs(
            x - start_x) > VIA_LENGTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2:
        # 检查各路径段是否与障碍物冲突
        condition3 = (
                not judge_DRC_violations(rect, seg_rect_s01) and
                not judge_DRC_violations(rect1, seg_rect_sP1) and
                not judge_DRC_violations(rect, seg_rect_s53) and
                not judge_DRC_violations(rect1, seg_rect_sP5) and
                not judge_DRC_violations(rect, via_s2) and
                not judge_DRC_violations(rect1, via_s3) and
                not judge_DRC_violations(rect1, via_s2) and
                not judge_DRC_violations(rect, via_s3)
        )
        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition3:
            s_path = seg_01 + seg_53
            t_path = seg_P1 + seg_P5
            return s_path, t_path, [(via2, 1), (via3, 1)], dict_node


    if abs(y - end_y) > VIA_WIDTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2 and abs(
            x - start_x) > VIA_LENGTH + LINE_SPACING_S:
        # 检查各路径段是否与障碍物冲突
        condition3 = (
                not judge_DRC_violations(rect, seg_rect_s01) and
                not judge_DRC_violations(rect1, seg_rect_sP1) and
                not judge_DRC_violations(rect, seg_rect_s53) and
                not judge_DRC_violations(rect, seg_rect_sP5) and
                not judge_DRC_violations(rect, via_s2) and
                not judge_DRC_violations(rect1, via_p) and
                not judge_DRC_violations(rect1, via_s2) and
                not judge_DRC_violations(rect, via_p)
        )
        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition3:
            s_path = seg_01 + seg_53 + seg_P5
            t_path = seg_P1
            return s_path, t_path, [(via2, 1), (p, 1)], dict_node
    if abs(y - end_y) > VIA_WIDTH + LINE_SPACING_S and abs(
            x - start_x) > VIA_LENGTH / 2 + LINE_SPACING_S + LINE_WIDTH_S / 2:
        # 检查各路径段是否与障碍物冲突
        condition3 = (
                not judge_DRC_violations(rect, seg_rect_s01) and
                not judge_DRC_violations(rect, seg_rect_sP1) and
                not judge_DRC_violations(rect, seg_rect_s53) and
                not judge_DRC_violations(rect1, seg_rect_sP5) and
                not judge_DRC_violations(rect, via_s3) and
                not judge_DRC_violations(rect1, via_p) and
                not judge_DRC_violations(rect1, via_s3) and
                not judge_DRC_violations(rect, via_p)
        )
        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition3:
            s_path = seg_01 + seg_53 + seg_P1
            t_path = seg_P5
            return s_path, t_path, [(via3, 1), (p, 1)], dict_node
    # 如果存在冲突，返回空结果
    return None, None, None, dict_node

def check_path_samey_all(start_x, start_y, x, x1, end_x, end_y, rect, rect1):
    # 定义路径上的节点
    p = (x, start_y)
    p1 =  (x1, start_y)
    start_x, end_x = min(start_x, end_x), max(start_x, end_x)
    node0 = (start_x, start_y)
    node1 = (end_x, end_y)

    seg1 = [] if node0 == p else [(node0, p)]
    seg2 = [] if p == p1 else [(p, p1)]
    seg3 = [] if p1 == node1 else [(p1, node1)]

    # 修改路径以避开障碍物
    seg_rect_s1 = path_ob_discriminatory(seg1)
    seg_rect_s2 = path_ob_discriminatory(seg2)
    seg_rect_s3 = path_ob_discriminatory(seg3)

    # 获取通孔矩形
    via_s1 = get_via_rect_s(p)
    via_s2 = get_via_rect_s(p1)



    if abs(x1 - x) >= VIA_LENGTH + LINE_SPACING_T:
        # 检查各路径段是否与障碍物冲突
        condition3 = (
                not judge_DRC_violations(rect, seg_rect_s1) and
                not judge_DRC_violations(rect1, seg_rect_s2) and
                not judge_DRC_violations(rect, seg_rect_s3) and
                not judge_DRC_violations(rect, via_s1) and
                not judge_DRC_violations(rect1, via_s1) and
                not judge_DRC_violations(rect, via_s2) and
                not judge_DRC_violations(rect1, via_s2)
        )
        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition3:
            s_path = seg1 + seg3
            t_path = seg2
            return s_path, t_path, [(p, 1), (p1, 1)]
    # 如果存在冲突，返回空结果
    return None, None, None


def check_path_samex_all(start_x, start_y, y, y1, end_x, end_y, rect, rect1):
    # 定义路径上的节点
    p = (start_x, y)
    p1 = (start_x, y1)
    start_y, end_y = min(start_y, end_y), max(start_y, end_y)
    node0 = (start_x, start_y)
    node1 = (end_x, end_y)

    seg1 = [] if node0 == p else [(node0, p)]
    seg2 = [] if p == p1 else [(p, p1)]
    seg3 = [] if p1 == node1 else [(p1, node1)]

    # 修改路径以避开障碍物
    seg_rect_s1 = path_ob_discriminatory(seg1)
    seg_rect_s2 = path_ob_discriminatory(seg2)
    seg_rect_s3 = path_ob_discriminatory(seg3)

    # 获取通孔矩形
    via_s1 = get_via_rect_s(p)
    via_s2 = get_via_rect_s(p1)

    if abs(y1 - y) >= VIA_WIDTH + LINE_SPACING_T:
        # 检查各路径段是否与障碍物冲突
        condition3 = (
                not judge_DRC_violations(rect, seg_rect_s1) and
                not judge_DRC_violations(rect1, seg_rect_s2) and
                not judge_DRC_violations(rect, seg_rect_s3) and
                not judge_DRC_violations(rect, via_s1) and
                not judge_DRC_violations(rect1, via_s1) and
                not judge_DRC_violations(rect, via_s2) and
                not judge_DRC_violations(rect1, via_s2)
        )
        # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
        if condition3:
            s_path = seg1 + seg3
            t_path = seg2
            return s_path, t_path, [(p, 1), (p1, 1)]
    # 如果存在冲突，返回空结果
    return None, None, None

def check_node_via(node1,node2,rect,rect1):

    via1 = get_via_rect_s(node1)
    via11 = get_via_rect_s_v(node1)

    via2 = get_via_rect_t(node2)
    via22 = get_via_rect_t_v(node2)
    condition1 = (
            not judge_DRC_violations(rect, via1) and
            not judge_DRC_violations(rect, via2) and
            not judge_DRC_violations(rect1, via1) and
            not judge_DRC_violations(rect1, via2)
    )
    # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
    if condition1:
        return [(node1, 1), (node2, 1)]

    condition2 = (
            not judge_DRC_violations(rect, via11) and
            not judge_DRC_violations(rect, via2) and
            not judge_DRC_violations(rect1, via11) and
            not judge_DRC_violations(rect1, via2)
    )
    # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
    if condition2:
        return [(node1, 0), (node2, 1)]

    condition3 = (
            not judge_DRC_violations(rect, via11) and
            not judge_DRC_violations(rect, via22) and
            not judge_DRC_violations(rect1, via11) and
            not judge_DRC_violations(rect1, via22)
    )
    # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
    if condition3:
        return [(node1, 0), (node2, 0)]

    condition4 = (
            not judge_DRC_violations(rect, via1) and
            not judge_DRC_violations(rect, via22) and
            not judge_DRC_violations(rect1, via1) and
            not judge_DRC_violations(rect1, via22)
    )
    # 如果所有路径段均无冲突，则返回组合后的路径和空的通孔列表
    if condition4:
        return [(node1, 1), (node2, 0)]

    return None
