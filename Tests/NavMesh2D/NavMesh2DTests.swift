import XCTest
@testable import NavMesh2D

class NavMesh2DTests: XCTestCase {
    func testExample() {
        // This is an example of a functional test case.
        // Use XCTAssert and related functions to verify your tests produce the correct results.
        //XCTAssertEqual(NavMesh2D().text, "Hello, World!")
        //print(NavMesh2D().text)
        let mesh = NavMesh2D()
        let pntA = Vector2D(x: 0.0, y: 0.0)
        let pntB = Vector2D(x: 1.0, y: 0.0)
        let pntC = Vector2D(x: 1.0, y: 1.0)
        let pntD = Vector2D(x: 2.0, y: 3.0)
        let pntE = Vector2D(x: 1.0, y: 4.0)
        let pntF = Vector2D(x: 0.0, y: 4.0)
        let triA = NavTriangle(id: 1, groupId: 1, point1: pntA, point2: pntB, point3: pntC)
        let triB = NavTriangle(id: 2, groupId: 1, point1: pntB, point2: pntD, point3: pntC)
        let triC = NavTriangle(id: 3, groupId: 1, point1: pntC, point2: pntD, point3: pntE)
        let triD = NavTriangle(id: 4, groupId: 1, point1: pntC, point2: pntE, point3: pntF)
        
        triA.neighbors[1] = 2
        triB.neighbors[1] = 3
        triB.neighbors[2] = 1
        triC.neighbors[0] = 2
        triC.neighbors[2] = 4
        triD.neighbors[0] = 3
        
        mesh.addTriangle(triangle: triA)
        mesh.addTriangle(triangle: triB)
        mesh.addTriangle(triangle: triC)
        mesh.addTriangle(triangle: triD)
        //mesh.addTriangle(triangle: NavTriangle(id: 1, groupId: 1, point1: pntE, point2: pnt, point3: pntC))
        
        let startP = WayPoint(triangleId: 1, point: Vector2D(x: 0.5, y: 0.5))
        let endP = WayPoint(triangleId: 4, point: Vector2D(x: 0.5, y: 3.8))
        var path = [WayPoint]()
        do {
            try mesh.findPath(startPos: endP, endPos: startP, path: &path, offset: 0.0)
        } catch {
            print(error)
        }
        
        for wayp in path {
            print("way point:", wayp.triangleId, wayp.point.x, wayp.point.y)
        }
    }


    static var allTests : [(String, (NavMesh2DTests) -> () throws -> Void)] {
        return [
            ("testExample", testExample),
        ]
    }
}
