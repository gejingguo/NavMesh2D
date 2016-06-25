//
//  NavTriangle.swift
//  NavMesh2D
//
//  Created by andyge on 16/6/25.
//
//

import Foundation

/// 寻路三角形
public class NavTriangle: Triangle {
    // 寻路相关参数
    var sessionId = -1
    var parentId = -1
    var isOpend = false
    
    // 评估相关
    var hValue = 0.0
    var gValue = 0.0
    var inWallIndex = -1
    var outWallIndex = -1
    
    public override init() {
        super.init()
    }
    
    public override init(id: Int, groupId: Int, point1: Vector2D, point2: Vector2D, point3: Vector2D) {
        super.init(id: id, groupId: groupId, point1: point1, point2: point2, point3: point3)
    }
    
    /// 重置导航数
    public func reset() {
        sessionId = -1
        parentId = -1
        isOpend = false
        
        // 评估相关
        hValue = 0.0
        gValue = 0.0
        inWallIndex = -1
        outWallIndex = -1
    }
    
    /// 设置当前三角形的穿入边
    public func setArrivedWall(neighborId: Int) {
        if neighborId < 0 {
            return
        }
        self.inWallIndex = getWallIndex(neighborId: neighborId)
    }
    
    /// 获得通过当前三角形的花费
    public func getCost(neighborId: Int) -> Double {
        let outWallIndex = getWallIndex(neighborId: neighborId)
        if inWallIndex == -1 {
            return 0.0
        }
        else if (inWallIndex != 0) {
            return wallDistances[1]
        }
        else if outWallIndex == 1 {
            return wallDistances[0]
        }
        else {
            return wallDistances[2]
        }
    }
    
    /// 计算三角形估价函数h
    /// 使用该三角形的中心点（3个顶点的平均值）到路径终点的x和y方向的距离
    public func calcHeuristic(endPos: Vector2D) {
        hValue = (center - endPos).length
    }
}
