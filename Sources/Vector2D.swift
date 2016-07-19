//
//  Vector2D.swift
//  NavMesh2D
//
//  Created by andyge on 16/6/24.
//
//

import Foundation

public struct Vector2D {
    var x = 0.0;
    var y = 0.0;
}

extension Vector2D {
    
    /// 长度平方
    public var lengthSquared: Double {
        get {
            return dot(left: self, right: self)
        }
    }
    
    /// 长度
    public var length: Double {
        get {
            return sqrt(lengthSquared)
        }
    }
    
    /// 标准化
    public mutating func normalized() {
        self = normalize(vector: self)
    }
    
    /// 重置
    public mutating func reset() {
        self.x = 0.0
        self.y = 0.0
    }
    
    /// 旋转角度
    public mutating func rotated(angle: Double) {
        self = rotate(vector: self, angle: angle)
    }
}

public func normalize(vector: Vector2D) -> Vector2D {
    let len = vector.length
    return Vector2D(x: vector.x/len, y: vector.y/len)
}

public func rotate(vector: Vector2D, angle: Double) -> Vector2D {
    let c = cos(angle)
    let s = sin(angle)
    return Vector2D(x: c*vector.x - s*vector.y, y: s*vector.y + c*vector.y)
}

public func dot(left: Vector2D, right: Vector2D) -> Double {
    return left.x * right.x + left.y * right.y
}

/// 叉乘
public func crossProduct(left: Vector2D, right: Vector2D) -> Double {
    return left.x * right.y - left.y * right.x
}

public func + (left: Vector2D, right: Vector2D) -> Vector2D {
    return Vector2D(x: left.x + right.x, y: left.y + right.y)
}

public func - (left: Vector2D, right: Vector2D) -> Vector2D {
    return Vector2D(x: left.x - right.x, y: left.y - right.y)
}

public func * (left: Vector2D, right: Vector2D) -> Vector2D {
    return Vector2D(x: left.x * right.x, y: left.y * right.y)
}

public func / (left: Vector2D, right: Vector2D) -> Vector2D {
    return Vector2D(x: left.x / right.x, y: left.y / right.y)
}

public prefix func - (vector: Vector2D) -> Vector2D {
    return Vector2D(x: -vector.x, y: -vector.y)
}

public func += ( left: inout Vector2D, right: Vector2D) {
    left = left + right
}

public func -= ( left: inout Vector2D, right: Vector2D) {
    left = left - right
}

public func *= ( left: inout Vector2D, right: Vector2D) {
    left = left * right
}

public func /= ( left: inout Vector2D, right: Vector2D) {
    left = left / right
}

public func == (left: Vector2D, right: Vector2D) -> Bool {
    return (left.x == right.x) && (left.y == right.y)
}

public func != (left: Vector2D, right: Vector2D) -> Bool {
    return !(left == right)
}

public func * (left: Vector2D, d: Double) -> Vector2D {
    return Vector2D(x: left.x * d, y: left.y * d)
}

