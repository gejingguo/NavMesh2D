//
//  Line.swift
//  NavMesh2D
//
//  Created by andyge on 16/6/25.
//
//

import Foundation

public enum PointSide {
    case Online
    case Left
    case Right
}

public enum LineCrossState {
    case Coline     // 外线口
    case Parallel   // 平行
    case Cross      // 相交
    case NotCross   // 不相交
}

public struct Line {
    public var start = Vector2D()
    public var end = Vector2D()
}

public func == (left: Line, right: Line) -> Bool {
    return left.start == right.start && left.end == right.end
}

public func != (left: Line, right: Line) -> Bool {
    return !(left == right)
}

public extension Line {
    /// 长度
    public var length: Double {
        get {
            let vec = end - start
            return vec.length
        }
    }
    
    /// 方向
    public var direction: Vector2D {
        get {
            return end - start
        }
    }
    
    /// 判断点于直线的关系
    public func classifyPoint(point: Vector2D) -> PointSide {
        if point == start || point == end {
            return PointSide.Online
        }
        let vecA = end - start
        let vecB = point - start
        
        let crossResult = crossProduct(left: vecA, right: vecB)
        if crossResult == 0.0 {
            return PointSide.Online
        } else if crossResult < 0.0 {
            return PointSide.Right
        } else {
            return PointSide.Left
        }
    }
}

