//
//  CameraCalibration.swift
//  Roboflow Starter Project
//
//  Created for Triangulation Prototype
//

import UIKit
import AVFoundation
import Vision

/// Represents the intrinsic parameters of a camera
struct CameraIntrinsics {
    // Focal length in pixels (x and y components)
    let focalLengthX: Double
    let focalLengthY: Double
    
    // Principal point (optical center) in pixels
    let principalPointX: Double
    let principalPointY: Double
    
    // Distortion coefficients (simplified)
    let distortionCoefficients: [Double]
    
    // Image dimensions
    let imageWidth: Int
    let imageHeight: Int
    
    // Device identifier
    let deviceId: String
    
    // Computed properties for easier access
    var focalLength: CGPoint {
        return CGPoint(x: focalLengthX, y: focalLengthY)
    }
    
    var principalPoint: CGPoint {
        return CGPoint(x: principalPointX, y: principalPointY)
    }
    
    /// Create default intrinsics based on device model
    static func defaultForDevice() -> CameraIntrinsics {
        let device = UIDevice.current
        let screenBounds = UIScreen.main.bounds
        let screenWidth = Int(screenBounds.width)
        let screenHeight = Int(screenBounds.height)
        
        // Default values based on device model
        // These are approximations and should be calibrated properly
        if device.model.contains("iPad Pro") {
            // iPad Pro 12.9" 5th gen approximation
            return CameraIntrinsics(
                focalLengthX: Double(screenWidth) * 1.2,
                focalLengthY: Double(screenWidth) * 1.2,
                principalPointX: Double(screenWidth) / 2.0,
                principalPointY: Double(screenHeight) / 2.0,
                distortionCoefficients: [0.0, 0.0, 0.0, 0.0, 0.0],
                imageWidth: screenWidth,
                imageHeight: screenHeight,
                deviceId: device.identifierForVendor?.uuidString ?? "unknown-ipad"
            )
        } else {
            // iPhone 12 Pro approximation
            return CameraIntrinsics(
                focalLengthX: Double(screenWidth) * 1.1,
                focalLengthY: Double(screenWidth) * 1.1,
                principalPointX: Double(screenWidth) / 2.0,
                principalPointY: Double(screenHeight) / 2.0,
                distortionCoefficients: [0.0, 0.0, 0.0, 0.0, 0.0],
                imageWidth: screenWidth,
                imageHeight: screenHeight,
                deviceId: device.identifierForVendor?.uuidString ?? "unknown-iphone"
            )
        }
    }
    
    /// Convert to dictionary for transmission
    func toDictionary() -> [String: Any] {
        return [
            "fx": focalLengthX,
            "fy": focalLengthY,
            "cx": principalPointX,
            "cy": principalPointY,
            "dist": distortionCoefficients,
            "width": imageWidth,
            "height": imageHeight,
            "device_id": deviceId
        ]
    }
    
    /// Create from dictionary
    static func fromDictionary(_ dict: [String: Any]) -> CameraIntrinsics? {
        guard
            let fx = dict["fx"] as? Double,
            let fy = dict["fy"] as? Double,
            let cx = dict["cx"] as? Double,
            let cy = dict["cy"] as? Double,
            let dist = dict["dist"] as? [Double],
            let width = dict["width"] as? Int,
            let height = dict["height"] as? Int,
            let deviceId = dict["device_id"] as? String
        else {
            return nil
        }
        
        return CameraIntrinsics(
            focalLengthX: fx,
            focalLengthY: fy,
            principalPointX: cx,
            principalPointY: cy,
            distortionCoefficients: dist,
            imageWidth: width,
            imageHeight: height,
            deviceId: deviceId
        )
    }
}

/// Represents the extrinsic parameters of a camera (position and orientation)
struct CameraExtrinsics {
    // Rotation matrix (3x3)
    let rotation: [[Double]]
    
    // Translation vector (3x1)
    let translation: [Double]
    
    // Device identifier
    let deviceId: String
    
    /// Initialize with rotation angles (in radians) and translation
    init(rotationAngles: [Double], translation: [Double], deviceId: String) {
        // Convert rotation angles to rotation matrix
        // Assuming rotation is [roll, pitch, yaw] in radians
        let roll = rotationAngles[0]
        let pitch = rotationAngles[1]
        let yaw = rotationAngles[2]
        
        // Create rotation matrices for each axis
        let Rx: [[Double]] = [
            [1.0, 0.0, 0.0],
            [0.0, cos(roll), -sin(roll)],
            [0.0, sin(roll), cos(roll)]
        ]
        
        let Ry: [[Double]] = [
            [cos(pitch), 0.0, sin(pitch)],
            [0.0, 1.0, 0.0],
            [-sin(pitch), 0.0, cos(pitch)]
        ]
        
        let Rz: [[Double]] = [
            [cos(yaw), -sin(yaw), 0.0],
            [sin(yaw), cos(yaw), 0.0],
            [0.0, 0.0, 1.0]
        ]
        
        // Combine rotation matrices (simplified for prototype)
        // In a real implementation, we would use proper matrix multiplication
        self.rotation = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]
        
        self.translation = translation
        self.deviceId = deviceId
    }
    
    /// Create identity extrinsics (camera at origin, no rotation)
    static func identity(deviceId: String) -> CameraExtrinsics {
        return CameraExtrinsics(
            rotation: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]
            ],
            translation: [0.0, 0.0, 0.0],
            deviceId: deviceId
        )
    }
    
    /// Convert to dictionary for transmission
    func toDictionary() -> [String: Any] {
        return [
            "R": rotation,
            "T": translation,
            "device_id": deviceId
        ]
    }
    
    /// Create from dictionary
    static func fromDictionary(_ dict: [String: Any]) -> CameraExtrinsics? {
        guard
            let r = dict["R"] as? [[Double]],
            let t = dict["T"] as? [Double],
            let deviceId = dict["device_id"] as? String
        else {
            return nil
        }
        
        return CameraExtrinsics(
            rotation: r,
            translation: t,
            deviceId: deviceId
        )
    }
}

/// Main class for camera calibration
class CameraCalibrator {
    // Camera parameters
    var intrinsics: CameraIntrinsics
    var extrinsics: CameraExtrinsics
    
    // Calibration state
    private var isCalibrated: Bool = false
    
    // Calibration object dimensions (in meters)
    private var calibrationObjectSize: Double = 0.2 // 20cm default
    
    init() {
        // Initialize with default parameters
        intrinsics = CameraIntrinsics.defaultForDevice()
        extrinsics = CameraExtrinsics.identity(deviceId: UIDevice.current.identifierForVendor?.uuidString ?? "unknown-device")
    }
    
    // Set intrinsics directly
    func setIntrinsics(_ newIntrinsics: CameraIntrinsics) {
        intrinsics = newIntrinsics
    }
    
    // Set extrinsics directly
    func setExtrinsics(_ newExtrinsics: CameraExtrinsics) {
        extrinsics = newExtrinsics
        isCalibrated = true
    }
    
    // Set extrinsics using translation and rotation vectors
    func setExtrinsics(translation: [Double], rotation: [Double]) {
        extrinsics = CameraExtrinsics(
            rotationAngles: rotation,
            translation: translation,
            deviceId: UIDevice.current.identifierForVendor?.uuidString ?? "unknown-device"
        )
        isCalibrated = true
    }
    
    /// Calibrate intrinsic parameters using a checkerboard pattern
    /// - Parameter pixelBuffer: Camera frame containing the checkerboard
    /// - Parameter completion: Callback with success/failure
    func calibrateIntrinsics(pixelBuffer: CVPixelBuffer, completion: @escaping (Bool) -> Void) {
        // In a real implementation, this would use VNDetectRectanglesRequest
        // or similar Vision framework calls to detect the checkerboard
        
        // For this prototype, we'll use the default parameters
        // and simulate a successful calibration
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            // In a real implementation, we would update the intrinsics here
            completion(true)
        }
    }
    
    /// Calibrate extrinsic parameters using a known object
    /// - Parameter pixelBuffer: Camera frame containing the calibration object
    /// - Parameter objectPoints: 3D coordinates of the calibration object corners
    /// - Parameter completion: Callback with success/failure
    func calibrateExtrinsics(pixelBuffer: CVPixelBuffer, objectPoints: [[Double]], completion: @escaping (Bool) -> Void) {
        // In a real implementation, this would detect the calibration object,
        // extract its corners, and solve the PnP problem
        
        // For this prototype, we'll simulate a successful calibration
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
            // In a real implementation, we would update the extrinsics here
            self.isCalibrated = true
            completion(true)
        }
    }
    
    /// Get the projection matrix for this camera
    /// - Returns: 3x4 projection matrix as a flattened array
    func getProjectionMatrix() -> [Double] {
        // P = K[R|t] where K is the intrinsic matrix, R is rotation, t is translation
        
        // Create the intrinsic matrix K
        let K = [
            intrinsics.focalLengthX, 0, intrinsics.principalPointX,
            0, intrinsics.focalLengthY, intrinsics.principalPointY,
            0, 0, 1
        ]
        
        // Flatten the rotation matrix
        let R = extrinsics.rotation.flatMap { $0 }
        
        // Translation vector
        let t = extrinsics.translation
        
        // Compute P = K[R|t]
        var P = [Double](repeating: 0.0, count: 12)
        
        // This is a simplified calculation - a real implementation would use matrix multiplication
        // For now, we'll return a basic projection matrix
        P[0] = K[0]
        P[1] = K[1]
        P[2] = K[2]
        P[4] = K[3]
        P[5] = K[4]
        P[6] = K[5]
        P[8] = K[6]
        P[9] = K[7]
        P[10] = K[8]
        
        return P
    }
    
    /// Convert a 2D point in image coordinates to a ray in 3D space
    /// - Parameter imagePoint: 2D point in image coordinates (pixels)
    /// - Returns: Ray direction in 3D space (normalized)
    func imagePointToRay(imagePoint: CGPoint) -> [Double] {
        // Normalize the image point
        let x = (Double(imagePoint.x) - intrinsics.principalPointX) / intrinsics.focalLengthX
        let y = (Double(imagePoint.y) - intrinsics.principalPointY) / intrinsics.focalLengthY
        
        // Create a ray in camera coordinates
        let rayCamera = [x, y, 1.0]
        
        // Transform to world coordinates using the camera's extrinsic parameters
        // This is a simplified calculation - a real implementation would use matrix multiplication
        
        // For now, just return the normalized ray in camera coordinates
        let magnitude = sqrt(rayCamera[0]*rayCamera[0] + rayCamera[1]*rayCamera[1] + rayCamera[2]*rayCamera[2])
        return [
            rayCamera[0] / magnitude,
            rayCamera[1] / magnitude,
            rayCamera[2] / magnitude
        ]
    }
    
    /// Estimate the distance to an object of known size
    /// - Parameter objectSizeMeters: The actual size of the object in meters
    /// - Parameter objectSizePixels: The size of the object in pixels
    /// - Returns: Estimated distance in meters
    func estimateDistance(objectSizeMeters: Double, objectSizePixels: Double) -> Double {
        // Simple pinhole camera model: distance = (objectSizeMeters * focalLengthPixels) / objectSizePixels
        let focalLengthPixels = (intrinsics.focalLengthX + intrinsics.focalLengthY) / 2.0
        return (objectSizeMeters * focalLengthPixels) / objectSizePixels
    }
} 