//
//  Triangulation.swift
//  Roboflow Starter Project
//
//  Created for Triangulation Prototype
//

import UIKit
import simd

/// Represents a 3D point with confidence
struct Point3D {
    let x: Double
    let y: Double
    let z: Double
    let confidence: Double
    
    /// Convert to dictionary for transmission
    func toDictionary() -> [String: Any] {
        return [
            "x": x,
            "y": y,
            "z": z,
            "confidence": confidence
        ]
    }
    
    /// Create from dictionary
    static func fromDictionary(_ dict: [String: Any]) -> Point3D? {
        guard
            let x = dict["x"] as? Double,
            let y = dict["y"] as? Double,
            let z = dict["z"] as? Double,
            let confidence = dict["confidence"] as? Double
        else {
            return nil
        }
        
        return Point3D(x: x, y: y, z: z, confidence: confidence)
    }
}

/// Represents a 2D detection with timestamp
struct Detection2D {
    let x: Double
    let y: Double
    let width: Double
    let height: Double
    let confidence: Double
    let timestamp: Double
    let deviceId: String
    
    /// Convert to dictionary for transmission
    func toDictionary() -> [String: Any] {
        return [
            "x": x,
            "y": y,
            "w": width,
            "h": height,
            "cf": confidence,
            "ts": timestamp,
            "device_id": deviceId
        ]
    }
    
    /// Create from dictionary
    static func fromDictionary(_ dict: [String: Any]) -> Detection2D? {
        guard
            let x = dict["x"] as? Double,
            let y = dict["y"] as? Double,
            let w = dict["w"] as? Double,
            let h = dict["h"] as? Double,
            let cf = dict["cf"] as? Double,
            let ts = dict["ts"] as? Double,
            let deviceId = dict["device_id"] as? String
        else {
            return nil
        }
        
        return Detection2D(
            x: x,
            y: y,
            width: w,
            height: h,
            confidence: cf,
            timestamp: ts,
            deviceId: deviceId
        )
    }
    
    /// Get the center point of the detection
    var center: CGPoint {
        return CGPoint(x: x, y: y)
    }
    
    /// Get the size of the object in pixels (average of width and height)
    var sizePixels: Double {
        return (width + height) / 2.0
    }
}

/// Main class for triangulation
class Triangulator {
    // Camera calibrators for each device
    private var calibrators: [String: CameraCalibrator] = [:]
    
    // Known object size (pickleball diameter in meters)
    private let pickleballDiameter: Double = 0.074 // 74mm
    
    // Maximum time difference for matching detections (in seconds)
    private let maxTimeDifference: Double = 0.1 // 100ms
    
    // Latest detections from each device
    private var latestDetections: [String: Detection2D] = [:]
    
    // Latest triangulated position
    private(set) var latestPosition: Point3D?
    
    init() {
        // Initialize with local device calibrator
        let localCalibrator = CameraCalibrator()
        let deviceId = UIDevice.current.identifierForVendor?.uuidString ?? "unknown-device"
        calibrators[deviceId] = localCalibrator
    }
    
    /// Add a camera calibrator for a device
    /// - Parameters:
    ///   - calibrator: The calibrator to add
    ///   - deviceId: The device identifier
    func addCalibrator(calibrator: CameraCalibrator, deviceId: String) {
        calibrators[deviceId] = calibrator
    }
    
    /// Add a detection from a device
    /// - Parameters:
    ///   - detection: The 2D detection
    ///   - deviceId: The device identifier
    /// - Returns: True if triangulation was performed
    func addDetection(detection: Detection2D) -> Bool {
        // Store the detection
        latestDetections[detection.deviceId] = detection
        
        // Check if we have detections from at least two devices
        if latestDetections.count >= 2 {
            // Check if the detections are close enough in time
            let timestamps = latestDetections.values.map { $0.timestamp }
            let minTimestamp = timestamps.min() ?? 0
            let maxTimestamp = timestamps.max() ?? 0
            
            if maxTimestamp - minTimestamp <= maxTimeDifference {
                // Perform triangulation
                triangulatePosition()
                return true
            }
        }
        
        return false
    }
    
    /// Triangulate the 3D position from the latest detections
    private func triangulatePosition() {
        // We need at least two detections and calibrators
        guard latestDetections.count >= 2 else { return }
        
        // Get the two most recent detections
        let sortedDetections = latestDetections.values.sorted { $0.timestamp > $1.timestamp }
        let detection1 = sortedDetections[0]
        let detection2 = sortedDetections[1]
        
        // Get the corresponding calibrators
        guard
            let calibrator1 = calibrators[detection1.deviceId],
            let calibrator2 = calibrators[detection2.deviceId]
        else { return }
        
        // For this prototype, we'll use a simplified triangulation approach
        // In a real implementation, we would use the DLT algorithm
        
        // Get rays from each camera
        let ray1 = calibrator1.imagePointToRay(imagePoint: detection1.center)
        let ray2 = calibrator2.imagePointToRay(imagePoint: detection2.center)
        
        // Get camera positions
        let camera1Position = calibrator1.extrinsics.translation
        let camera2Position = calibrator2.extrinsics.translation
        
        // Estimate distances using the known ball size
        let distance1 = calibrator1.estimateDistance(
            objectSizeMeters: pickleballDiameter,
            objectSizePixels: detection1.sizePixels
        )
        
        let distance2 = calibrator2.estimateDistance(
            objectSizeMeters: pickleballDiameter,
            objectSizePixels: detection2.sizePixels
        )
        
        // Simple triangulation: average the two position estimates
        let position1 = [
            camera1Position[0] + ray1[0] * distance1,
            camera1Position[1] + ray1[1] * distance1,
            camera1Position[2] + ray1[2] * distance1
        ]
        
        let position2 = [
            camera2Position[0] + ray2[0] * distance2,
            camera2Position[1] + ray2[1] * distance2,
            camera2Position[2] + ray2[2] * distance2
        ]
        
        // Average the positions
        let x = (position1[0] + position2[0]) / 2.0
        let y = (position1[1] + position2[1]) / 2.0
        let z = (position1[2] + position2[2]) / 2.0
        
        // Calculate confidence based on detection confidences and time difference
        let confidenceAvg = (detection1.confidence + detection2.confidence) / 2.0
        let timeDiff = abs(detection1.timestamp - detection2.timestamp)
        let timeConfidence = 1.0 - (timeDiff / maxTimeDifference)
        let confidence = confidenceAvg * timeConfidence
        
        // Store the result
        latestPosition = Point3D(x: x, y: y, z: z, confidence: confidence)
    }
    
    /// Get the latest triangulated position
    /// - Returns: The latest position, or nil if not available
    func getLatestPosition() -> Point3D? {
        return latestPosition
    }
    
    /// Clear all stored detections
    func clearDetections() {
        latestDetections.removeAll()
        latestPosition = nil
    }
} 