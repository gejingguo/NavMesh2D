//
//  Rect.swift
//  NavMesh2D
//
//  Created by andyge on 16/6/25.
//
//

import Foundation

public struct Size {
    var width = 0.0
    var height = 0.0
}

public struct Rect {
    var origin = Vector2D()
    var size = Size()
}

public extension Rect {
    public var center: Vector2D {
        get {
            let centerX = origin.x + (size.width / 2)
            let centerY = origin.y + (size.height / 2)
            return Vector2D(x: centerX, y: centerY)
        }
        set(newCenter) {
            origin.x = newCenter.x - (size.width / 2)
            origin.y = newCenter.y - (size.height / 2)
        }
    }
    
    public func contains(point: Vector2D) -> Bool {
        let x = origin.x + size.width
        let y = origin.y + size.height
        return (point.x >= origin.x && point.x <= x && point.y >= origin.y && point.y <= y)
    }
}
