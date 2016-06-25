//
//  Triangle.swift
//  NavMesh2D
//
//  Created by andyge on 16/6/25.
//
//

import Foundation

/// 三角形
public class Triangle {
    public var id = -1
    public var groupId = -1
    public var points = [Vector2D](repeating: Vector2D(), count: 3)
    public var neighbors = [Int](repeating: -1, count: 3)
    
    var center = Vector2D()
    var box = Rect()
    var wallDistances = [Double](repeating: 0.0, count: 3)
    
    public init() {
        
    }
    
    public init(id: Int, groupId: Int, point1: Vector2D, point2: Vector2D, point3: Vector2D) {
        self.id = id
        self.groupId = groupId
        self.points[0] = point1
        self.points[1] = point2
        self.points[2] = point3
        
        // 计算中心点
        center.x = (points[0].x + points[1].x + points[2].x)/3
        center.y = (points[0].y + points[1].y + points[2].y)/3

        // 计算相邻俩点中点距离
        var wallMidPoints = [Vector2D](repeating: Vector2D(), count: 3)
        wallMidPoints[0] = Vector2D(x: (points[0].x + points[1].x)/2, y: (points[0].y + points[1].y)/2)
        wallMidPoints[1] = Vector2D(x: (points[1].x + points[2].x)/2, y: (points[1].y + points[2].y)/2)
        wallMidPoints[2] = Vector2D(x: (points[2].x + points[0].x)/2, y: (points[2].y + points[0].y)/2)
        
        wallDistances[0] = (wallMidPoints[0] - wallMidPoints[1]).length
        wallDistances[1] = (wallMidPoints[1] - wallMidPoints[2]).length
        wallDistances[2] = (wallMidPoints[2] - wallMidPoints[0]).length
        
        // 计算包围盒
        calcBoxCollider()
    }
    
    /// 计算包围盒
    func calcBoxCollider() {
        if points[0] == points[1] || points[1] == points[2] || points[2] == points[0] {
            fatalError("triangle not valid.")
            //return
        }
        
        var xMin = points[0].x
        var xMax = xMin
        var yMin = points[0].y
        var yMax = yMin
        for i in 1..<3 {
            if points[i].x < xMin {
                xMin = points[i].x
            }
            else if points[i].x > xMax {
                xMax = points[i].x
            }
            else if points[i].y < yMin {
                yMin = points[i].y
            }
            else if points[i].y > yMax {
                yMax = points[i].y
            }
        }
        
        self.box.origin = Vector2D(x: xMin, y: yMin)
        self.box.size.width = xMax - xMin
        self.box.size.height = yMax - yMin
    }
    
    /// 获取索引对应的边
    public func getSide(index: Int) -> Line? {
        switch index {
        case 0:
            return Line(start: points[0], end: points[1])
        case 1:
            return Line(start: points[1], end: points[2])
        case 2:
            return Line(start: points[2], end: points[0])
        default:
            return nil
        }
    }
    
    /// 检查点是否在三角形中（边上也算）
    public func contains(point: Vector2D) -> Bool {
        if !self.box.contains(point: point) {
            return false;
        }
        
        guard let resultA = getSide(index: 0)?.classifyPoint(point: point) else {
            return false
        }
        guard let resultB = getSide(index: 1)?.classifyPoint(point: point) else {
            return false
        }
        guard let resultC = getSide(index: 2)?.classifyPoint(point: point) else {
            return false
        }
        
        if resultA == PointSide.Online || resultB == PointSide.Online || resultC == PointSide.Online {
            return true
        }
        else if resultA == PointSide.Right && resultB == PointSide.Right && resultC == PointSide.Right {
            return true
        }
        else {
            return false
        }
    }
    
    /// 检查是否是邻居三角形
    public func isNeighbor(triangle: Triangle) -> Bool {
        for i in 0..<3 {
            if neighbors[i] == triangle.id {
                return true;
            }
        }
        return false;
    }
    
    /// 获取邻居三角形边索引
    public func getWallIndex(neighborId: Int) -> Int {
        for i in 0..<3 {
            if neighbors[i] == neighborId {
                return i
            }
        }
        return -1
    }
}
