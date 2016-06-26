//
//  NavMesh2D.swift
//  NavMesh2D
//
//  Created by andyge on 16/6/24.
//
//

import Foundation

/// 寻路错误
public enum NavError: ErrorProtocol {
    case Failed
    case NoMeshData
    case NoStartOrEndTri
    case NavIDNotMatch
    case NotFoundPath
    case CanNotGetNextWayPoint
    case GroupNotMatch
    case NoCrossPoint
    case FixPointFailed
}

/// 寻路点
public struct WayPoint {
    var triangleId = -1
    var point = Vector2D()
}

/// 导航网格
/// 三角形的三个顶点顺序必须是顺时针
public class NavMesh2D {
    /// 三角形列表
    var triangles = [NavTriangle](repeating: NavTriangle(), count: 1000)
    
    public init() {
    }
    
    /// 从Json文件加载 
    public func load(from: String) throws {
        //JSONSerialization.jsonObject(with: <#T##Data#>, options: <#T##JSONSerialization.ReadingOptions#>)
    }
    
    /// 重置导航数据
    public func resetData() {
        for tri in self.triangles {
            tri.reset()
        }
    }
    
    /// 获取指定ID三角形
    public func getTriangle(id: Int) -> NavTriangle? {
        if id >= triangles.count {
            return nil
        }
        if id != triangles[id].id {
            return nil
        }
        return triangles[id]
    }
    
    /// 添加三角形
    public func addTriangle(triangle: NavTriangle) {
        if triangle.id < 0 || triangle.id >= triangles.count {
            fatalError("invalid triangle add.")
        }
        //triangles.reserveCapacity(triangle.id + 1)
        triangles[triangle.id] = triangle
    }
    
    /// 寻路
    public func findPath(startPos: WayPoint, endPos: WayPoint, path: inout [WayPoint], offset: Double = 0.0) throws {
        if triangles.isEmpty {
            throw NavError.NoMeshData
        }
        
        // 检查起点，终点有效性
        guard let startTri = getTriangle(id: startPos.triangleId) else {
            throw NavError.NoStartOrEndTri
        }
        if !startTri.contains(point: startPos.point) {
            throw NavError.NoStartOrEndTri
        }
        guard let endTri = getTriangle(id: endPos.triangleId) else {
            throw NavError.NoStartOrEndTri
        }
        if !endTri.contains(point: endPos.point) {
            throw NavError.NoStartOrEndTri
        }
        
        resetData()
        
        var navPath = [NavTriangle]()
        try findTrianglePath(startPos: startPos, endPos: endPos, path: &navPath, offset: offset)
        // 保证从起点到终点的顺序
        navPath.reverse()
        //for tri in navPath {
        //    print("path tri:", tri.id)
        //}
        try createWayPoints(startPos: startPos, endPos: endPos, navPath: navPath, offset: offset, wayPath: &path)
    }
    
    /// 寻路路径三角形
    public func findTrianglePath(startPos: WayPoint, endPos: WayPoint, path: inout [NavTriangle], offset: Double) throws {
        guard let startTri = self.getTriangle(id: startPos.triangleId) else {
            throw NavError.NoStartOrEndTri
        }
        guard let endTri = self.getTriangle(id: endPos.triangleId) else {
            throw NavError.NoStartOrEndTri
        }
        
        // 如果起点和终点是同一个三角形
        if startPos.triangleId == endPos.triangleId {
            path.append(startTri)
            return
        }
        
        // 如果起点和终点相邻
        if startTri.isNeighbor(triangle: endTri) {
            path.append(endTri)
            path.append(startTri)
            return
        }
        
        // A* 寻路
        let pathSessionId = 1
        var foundPath = false
        var openList = [NavTriangle]()
        var closeList = [NavTriangle]()
        
        startTri.sessionId = pathSessionId
        openList.append(startTri)
        while openList.count > 0 {
            // 1. 把当前节点从开放列表删除，放入封闭列表
            let curTri = openList.removeLast()
            closeList.append(curTri)
            
            // 已经找到目的地
            if curTri.id == endTri.id {
                foundPath = true
                break
            }
            
            // 2. 对当前节点相邻的每个节点依次执行以下步骤
            for i in 0..<3 {
                let neighborId = curTri.neighbors[i]
                // 3.如果该相邻节点不可通行，不做操作，检查下一个节点
                if neighborId < 0 {
                    continue
                }
                guard let neighborTri = self.getTriangle(id: neighborId) else {
                    throw NavError.NavIDNotMatch
                }
                if neighborTri.groupId == startTri.groupId {
                    if neighborTri.sessionId != pathSessionId {
                        // 
                        let sideIndex = neighborTri.getWallIndex(neighborId: curTri.id)
                        if let neighborSide = neighborTri.getSide(index: sideIndex) {
                            if neighborSide.length >= offset {
                                // 4. 如果该相邻节点不在开放列表中,则将该节点添加到开放列表中,
                                //    并将该相邻节点的父节点设为当前节点,同时保存该相邻节点的G和F值;
                                neighborTri.sessionId = pathSessionId
                                neighborTri.parentId = curTri.id
                                neighborTri.isOpend = true
                                
                                // 计算启发值h
                                neighborTri.calcHeuristic(endPos: endPos.point)
                                // 计算三角形花费g
                                //neighborTri.SetGValue(currNode.GetGValue() + currNode.GetCost(neighborTri.GetID()) );
                                neighborTri.gValue = curTri.gValue + curTri.getCost(neighborId: neighborTri.id)
                                
                                //放入开放列表并排序
                                openList.append(neighborTri);
                                openList.sort {
                                    (tri1, tri2) -> Bool in
                                    return tri1.hValue < tri2.hValue
                                }
                                
                                //保存穿入边
                                //neighborTri.SetArrivalWall(currNode.GetID());
                                neighborTri.setArrivedWall(neighborId: curTri.id)
                            }
                        }
                    }
                    else {
                        // 5. 如果该相邻节点在开放列表中,
                        //    则判断若经由当前节点到达该相邻节点的G值是否小于原来保存的G值,
                        //    若小于,则将该相邻节点的父节点设为当前节点,并重新设置该相邻节点的G和F值
                        if neighborTri.isOpend {
                            if neighborTri.gValue + neighborTri.getCost(neighborId: curTri.id) < curTri.gValue {
                                curTri.gValue = neighborTri.gValue + neighborTri.getCost(neighborId: curTri.id)
                                curTri.parentId = neighborTri.id
                                curTri.setArrivedWall(neighborId: neighborTri.id)
                            }
                        } else {
                            continue
                        }
                    }
                }
            }
        }
        
        if closeList.count != 0 {
            var pathTri = closeList.removeLast()
            path.append(pathTri)
            while pathTri.parentId != -1 {
                if let parentTri = self.getTriangle(id: pathTri.parentId) {
                    path.append(parentTri)
                    pathTri = parentTri
                }
            }
        }
        if !foundPath {
            throw NavError.NotFoundPath
        }
    }
    
    /// 生成路径
    func createWayPoints(startPos: WayPoint, endPos: WayPoint, navPath: [NavTriangle], offset: Double, wayPath: inout [WayPoint]) throws {
        if navPath.count == 0 {
            throw NavError.Failed
        }
        
        // 保存出边编号
        for i in 0 ..< navPath.count {
            let tri = navPath[i]
            if i != (navPath.count - 1) {
                let nextTri = navPath[i + 1]
                tri.outWallIndex = tri.getWallIndex(neighborId: nextTri.id)
                //print("tri out wall index,", tri.id, tri.outWallIndex)
            }
        }
        
        wayPath.append(startPos)
        
        // 起点和终点在同一三角形中
        if navPath.count == 1 {
            wayPath.append(endPos)
            return
        }
        
        var way = startPos
        while way.point != endPos.point {
            guard let wayp = getFurthestWayPoint(way: way, navPath: navPath, endPos: endPos, offset: offset) else {
            //if way == nil {
                throw NavError.CanNotGetNextWayPoint
            }
            way = wayp
            wayPath.append(way)
        }
    }
    
    /// 根据拐点计算法获得导航网格的下一个拐点
    func getFurthestWayPoint(way: WayPoint, navPath: [NavTriangle], endPos: WayPoint, offset: Double) -> WayPoint? {
        guard var curTri = getTriangle(id: way.triangleId) else {
            return nil
        }
        let curPnt = way.point
        var lastTriA = curTri
        var lastTriB = curTri
        
        var startIndex = -1
        for i in 0 ..< navPath.count {
            if navPath[i].id == curTri.id {
                startIndex = i
                break
            }
        }
        if startIndex < 0 {
            return nil
        }
        
        guard var outSide = curTri.getSide(index: curTri.outWallIndex) else {
            return nil
        }
        
        var lastPntA = outSide.start
        var lastPntB = outSide.end
        var lastLineA = Line(start: curPnt, end: lastPntA)
        var lastLineB = Line(start: curPnt, end: lastPntB)
        
        var testPntA = Vector2D()
        var testPntB = Vector2D()
        for i in startIndex + 1 ..< navPath.count {
            curTri = navPath[i]
            
            if i == navPath.count - 1 {
                testPntA = endPos.point
                testPntB = endPos.point
            } else {
                outSide = curTri.getSide(index: curTri.outWallIndex)!
                testPntA = outSide.start
                testPntB = outSide.end
            }
            
            if lastPntA != testPntA {
                if lastLineB.classifyPoint(point: testPntA) == PointSide.Right {
                    return WayPoint(triangleId: lastTriB.id, point: lastPntB)
                }
                else if lastLineA.classifyPoint(point: testPntA) != PointSide.Left {
                    lastPntA = testPntA
                    lastTriA = curTri
                    // 
                    lastLineA = Line(start: lastLineA.start, end: lastPntA)
                }
            }
            
            if lastPntB != testPntB {
                if lastLineA.classifyPoint(point: testPntB) == PointSide.Left {
                    return WayPoint(triangleId: lastTriA.id, point: lastPntA)
                }
                else if lastLineB.classifyPoint(point: testPntB) != PointSide.Right {
                    lastPntB = testPntB
                    lastTriB = curTri
                    lastLineB = Line(start: lastLineB.start, end: lastPntB)
                }
            }
        }
        
        // 到达终点
        let nextWay = endPos //WayPoint(triangleId: navPath.last?.id, point: endPos)
        return nextWay
    }

    
}
