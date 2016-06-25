//
//  Circle.swift
//  NavMesh2D
//
//  Created by andyge on 16/6/25.
//
//

import Foundation

public struct Circle {
    var center = Vector2D()
    var radius = 0.0
}

public extension Circle {
    public func contains(point: Vector2D) -> Bool {
        let vec = point - self.center
        return vec.length <= radius
    }
}
