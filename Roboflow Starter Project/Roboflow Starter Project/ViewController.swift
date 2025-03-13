//
//  ViewController.swift
//  Roboflow Starter Project
//
//  Created by Nicholas Arner on 9/11/22.
//
// NOTE: To use MultipeerConnectivity, you need to add the following to your Info.plist:
// Key: NSLocalNetworkUsageDescription
// Value: This app uses the local network to connect with nearby devices for sharing detection data.
// Key: NSBonjourServices
// Value: _roboflow-mcp._tcp

import UIKit
import AVFoundation
import Vision
import Roboflow
import MultipeerConnectivity
import Foundation

// Minimal implementations of required types
// These should be removed once the Triangulation.swift and CameraCalibration.swift files
// are properly added to the Xcode project target

/// Represents a 3D point with confidence
struct Point3D {
    let x: Double
    let y: Double
    let z: Double
    let confidence: Double
    
    func toDictionary() -> [String: Any] {
        return ["x": x, "y": y, "z": z, "confidence": confidence]
    }
    
    static func fromDictionary(_ dict: [String: Any]) -> Point3D? {
        guard let x = dict["x"] as? Double,
              let y = dict["y"] as? Double,
              let z = dict["z"] as? Double,
              let confidence = dict["confidence"] as? Double else {
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
    
    var center: CGPoint { return CGPoint(x: x, y: y) }
    var sizePixels: Double { return (width + height) / 2.0 }
    
    func toDictionary() -> [String: Any] {
        return [
            "x": x, "y": y, "w": width, "h": height,
            "cf": confidence, "ts": timestamp, "device_id": deviceId
        ]
    }
    
    static func fromDictionary(_ dict: [String: Any]) -> Detection2D? {
        guard let x = dict["x"] as? Double,
              let y = dict["y"] as? Double,
              let w = dict["w"] as? Double,
              let h = dict["h"] as? Double,
              let cf = dict["cf"] as? Double,
              let ts = dict["ts"] as? Double,
              let deviceId = dict["device_id"] as? String else {
            return nil
        }
        return Detection2D(x: x, y: y, width: w, height: h, confidence: cf, timestamp: ts, deviceId: deviceId)
    }
}

/// Minimal camera intrinsics struct
struct CameraIntrinsics {
    let focalLengthX: Double
    let focalLengthY: Double
    let principalPointX: Double
    let principalPointY: Double
    
    func toDictionary() -> [String: Any] {
        return ["fx": focalLengthX, "fy": focalLengthY, "cx": principalPointX, "cy": principalPointY]
    }
    
    static func fromDictionary(_ dict: [String: Any]) -> CameraIntrinsics? {
        guard let fx = dict["fx"] as? Double,
              let fy = dict["fy"] as? Double,
              let cx = dict["cx"] as? Double,
              let cy = dict["cy"] as? Double else {
            return nil
        }
        return CameraIntrinsics(focalLengthX: fx, focalLengthY: fy, principalPointX: cx, principalPointY: cy)
    }
}

/// Minimal camera extrinsics struct
struct CameraExtrinsics {
    let rotation: [[Double]]
    let translation: [Double]
    
    func toDictionary() -> [String: Any] {
        return ["R": rotation, "T": translation]
    }
    
    static func fromDictionary(_ dict: [String: Any]) -> CameraExtrinsics? {
        guard let rotation = dict["R"] as? [[Double]],
              let translation = dict["T"] as? [Double] else {
            return nil
        }
        return CameraExtrinsics(rotation: rotation, translation: translation)
    }
}

/// Minimal camera calibrator class
class CameraCalibrator {
    var intrinsics: CameraIntrinsics
    var extrinsics: CameraExtrinsics
    
    init() {
        // Default values
        intrinsics = CameraIntrinsics(focalLengthX: 1000, focalLengthY: 1000, principalPointX: 320, principalPointY: 240)
        extrinsics = CameraExtrinsics(rotation: [[1,0,0],[0,1,0],[0,0,1]], translation: [0,0,0])
    }
    
    func setIntrinsics(_ newIntrinsics: CameraIntrinsics) {
        intrinsics = newIntrinsics
    }
    
    func setExtrinsics(_ newExtrinsics: CameraExtrinsics) {
        extrinsics = newExtrinsics
    }
    
    func setExtrinsics(translation: [Double], rotation: [Double]) {
        extrinsics = CameraExtrinsics(rotation: [[1,0,0],[0,1,0],[0,0,1]], translation: translation)
    }
    
    func imagePointToRay(imagePoint: CGPoint) -> [Double] {
        // Convert image point to normalized device coordinates
        let normalizedX = (Double(imagePoint.x) - intrinsics.principalPointX) / intrinsics.focalLengthX
        let normalizedY = (Double(imagePoint.y) - intrinsics.principalPointY) / intrinsics.focalLengthY
        
        // Create a ray direction vector (in camera coordinates)
        let rayX = normalizedX
        let rayY = normalizedY
        let rayZ = 1.0 // Forward direction
        
        // Normalize the ray direction
        let magnitude = sqrt(rayX * rayX + rayY * rayY + rayZ * rayZ)
        let normalizedRayX = rayX / magnitude
        let normalizedRayY = rayY / magnitude
        let normalizedRayZ = rayZ / magnitude
        
        // Apply camera rotation to get the ray in world coordinates
        // For simplicity, we'll use the identity rotation if no rotation is specified
        let rotatedRay = applyRotation(
            vector: [normalizedRayX, normalizedRayY, normalizedRayZ],
            rotation: extrinsics.rotation
        )
        
        return rotatedRay
    }
    
    private func applyRotation(vector: [Double], rotation: [[Double]]) -> [Double] {
        // Apply 3x3 rotation matrix to a 3D vector
        guard rotation.count == 3 && rotation[0].count == 3 else {
            // Return the original vector if rotation matrix is invalid
            return vector
        }
        
        let x = vector[0]
        let y = vector[1]
        let z = vector[2]
        
        let rotatedX = rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z
        let rotatedY = rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z
        let rotatedZ = rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z
        
        return [rotatedX, rotatedY, rotatedZ]
    }
    
    func estimateDistance(objectSizeMeters: Double, objectSizePixels: Double) -> Double {
        // Get the pixels per meter value from calibration, or use a default if not available
        let pixelsPerMeter = UserDefaults.standard.double(forKey: "pixelsPerMeter")
        if pixelsPerMeter > 0 {
            return objectSizePixels / pixelsPerMeter
        } else {
            // Fallback to the simplified calculation if no calibration data
            return (objectSizeMeters * 1000) / objectSizePixels
        }
    }
}

/// Minimal triangulator class
class Triangulator {
    private var calibrators: [String: CameraCalibrator] = [:]
    private var latestDetections: [String: Detection2D] = [:]
    private(set) var latestPosition: Point3D?
    
    // Constants for triangulation
    private let ballDiameterMeters: Double = 0.0508 // Standard pickleball diameter (2 inches)
    private let maxTimeDifference: Double = 0.1 // Maximum time difference between detections (100ms)
    
    init() {
        let deviceId = UIDevice.current.identifierForVendor?.uuidString ?? "unknown-device"
        calibrators[deviceId] = CameraCalibrator()
    }
    
    func addCalibrator(calibrator: CameraCalibrator, deviceId: String) {
        calibrators[deviceId] = calibrator
    }
    
    func addDetection(detection: Detection2D) -> Bool {
        // Store the detection
        latestDetections[detection.deviceId] = detection
        
        // We need at least 2 detections from different devices to triangulate
        if latestDetections.count >= 2 {
            // Filter detections that are too old
            let currentTime = detection.timestamp
            let validDetections = latestDetections.filter { $0.value.timestamp >= currentTime - maxTimeDifference }
            
            // If we still have at least 2 valid detections, perform triangulation
            if validDetections.count >= 2 {
                // Get the first two detections (from different devices)
                let detectionArray = Array(validDetections.values)
                let detection1 = detectionArray[0]
                let detection2 = detectionArray[1]
                
                // Debug: Print available calibrators and detection device IDs
                print("Available calibrators for devices: \(calibrators.keys.joined(separator: ", "))")
                print("Detection 1 device ID: \(detection1.deviceId)")
                print("Detection 2 device ID: \(detection2.deviceId)")
                
                // Get the calibrators for these devices
                guard let calibrator1 = calibrators[detection1.deviceId],
                      let calibrator2 = calibrators[detection2.deviceId] else {
                    print("Missing calibrators for devices. Available: \(calibrators.keys.joined(separator: ", "))")
                    print("Needed: \(detection1.deviceId) and \(detection2.deviceId)")
                    return false
                }
                
                // Perform triangulation
                latestPosition = triangulatePosition(
                    detection1: detection1,
                    detection2: detection2,
                    calibrator1: calibrator1,
                    calibrator2: calibrator2
                )
                
                return latestPosition != nil
            }
        }
        
        return false
    }
    
    private func triangulatePosition(
        detection1: Detection2D,
        detection2: Detection2D,
        calibrator1: CameraCalibrator,
        calibrator2: CameraCalibrator
    ) -> Point3D? {
        print("Triangulating with detections from devices: \(detection1.deviceId) and \(detection2.deviceId)")
        print("Detection 1: x=\(detection1.x), y=\(detection1.y), size=\(detection1.sizePixels), conf=\(detection1.confidence)")
        print("Detection 2: x=\(detection2.x), y=\(detection2.y), size=\(detection2.sizePixels), conf=\(detection2.confidence)")
        
        // Convert image points to rays in 3D space
        let ray1 = calibrator1.imagePointToRay(imagePoint: detection1.center)
        let ray2 = calibrator2.imagePointToRay(imagePoint: detection2.center)
        
        print("Ray 1: \(ray1)")
        print("Ray 2: \(ray2)")
        
        // Get camera positions (origins of the rays)
        let origin1 = calibrator1.extrinsics.translation
        let origin2 = calibrator2.extrinsics.translation
        
        print("Camera 1 position: \(origin1)")
        print("Camera 2 position: \(origin2)")
        
        // For a simple implementation, we'll use a basic triangulation approach
        // In a real implementation, you would use a more sophisticated algorithm
        
        // Estimate distances based on the known ball size
        let distance1 = calibrator1.estimateDistance(
            objectSizeMeters: ballDiameterMeters,
            objectSizePixels: detection1.sizePixels
        )
        
        let distance2 = calibrator2.estimateDistance(
            objectSizeMeters: ballDiameterMeters,
            objectSizePixels: detection2.sizePixels
        )
        
        print("Estimated distance 1: \(distance1) meters")
        print("Estimated distance 2: \(distance2) meters")
        
        // Calculate 3D positions from each camera
        let pos1X = origin1[0] + ray1[0] * distance1
        let pos1Y = origin1[1] + ray1[1] * distance1
        let pos1Z = origin1[2] + ray1[2] * distance1
        
        let pos2X = origin2[0] + ray2[0] * distance2
        let pos2Y = origin2[1] + ray2[1] * distance2
        let pos2Z = origin2[2] + ray2[2] * distance2
        
        print("Position from camera 1: (\(pos1X), \(pos1Y), \(pos1Z))")
        print("Position from camera 2: (\(pos2X), \(pos2Y), \(pos2Z))")
        
        // Average the two positions (simple approach)
        let avgX = (pos1X + pos2X) / 2.0
        let avgY = (pos1Y + pos2Y) / 2.0
        let avgZ = (pos1Z + pos2Z) / 2.0
        
        // Calculate confidence based on time difference and detection confidences
        let timeDiff = abs(detection1.timestamp - detection2.timestamp)
        let timeConfidence = 1.0 - (timeDiff / maxTimeDifference)
        let detectionConfidence = (detection1.confidence + detection2.confidence) / 2.0
        let confidence = timeConfidence * detectionConfidence
        
        print("Final triangulated position: (\(avgX), \(avgY), \(avgZ)) with confidence \(confidence)")
        
        // Create and return the 3D point
        return Point3D(x: avgX, y: avgY, z: avgZ, confidence: confidence)
    }
    
    func getLatestPosition() -> Point3D? {
        return latestPosition
    }
    
    func clearDetections() {
        latestDetections.removeAll()
        latestPosition = nil
    }
    
    func getAvailableCalibrators() -> [String] {
        return Array(calibrators.keys)
    }
}

var API_KEY = "dJ0Ql7NDNI5ZIJFAdLsB"

extension Array where Element == CGPoint {
    /// Returns true if a bounce is detected at the end of this array.
    /// Simple logic: if the y-direction changed from downward to upward.
    func detectBounce() -> Bool {
        // Need at least 3 points
        guard self.count >= 3 else { return false }
        
        let count = self.count
        let p1 = self[count - 3]
        let p2 = self[count - 2]
        let p3 = self[count - 1]
        
        let dy1 = p2.y - p1.y   // movement from p1 -> p2
        let dy2 = p3.y - p2.y   // movement from p2 -> p3
        
        // "Bounce" if it was moving downward (dy1 > 0) and then upward (dy2 < 0).
        // Depending on iOS coords, you may need to invert or tweak the logic.
        return (dy1 > 0 && dy2 < 0)
    }
}


class ViewController: UIViewController, AVCaptureVideoDataOutputSampleBufferDelegate, MCSessionDelegate, MCNearbyServiceAdvertiserDelegate, MCNearbyServiceBrowserDelegate, MCBrowserViewControllerDelegate {
    
    var bufferSize: CGSize = .zero
    var rootLayer: CALayer! = nil
    
    private var detectionOverlay: CALayer! = nil
    var currentPixelBuffer: CVPixelBuffer!
    
    private let captureSession = AVCaptureSession()
    private var previewLayer: AVCaptureVideoPreviewLayer! = nil
    private let videoDataOutput = AVCaptureVideoDataOutput()
    private let videoDataOutputQueue = DispatchQueue(label: "VideoDataOutput", qos: .userInitiated, attributes: [], autoreleaseFrequency: .workItem)
    
    @IBOutlet weak private var previewView: UIView!
    @IBOutlet weak var fpsLabel: UILabel!
    
    //Initialize the Roboflow SDK
    let rf = RoboflowMobile(apiKey: API_KEY)
    var roboflowModel: RFObjectDetectionModel!
    
    // Store the trajectory of the ball (center points). We'll only track ONE ball for simplicity.
    var ballTrajectory: [CGPoint] = []
    
    // Controls how long to display a bounce annotation
    var bounceFramesRemaining: Int = 0
    
    // If you want to store the old direction to detect bounces:
    var previousVerticalDirections: [CGFloat] = []
    
    // MCP Properties
    private let serviceType = "roboflow-mcp"
    private var myPeerId: MCPeerID!
    private var mcSession: MCSession!
    private var mcAdvertiser: MCNearbyServiceAdvertiser!
    private var mcBrowser: MCNearbyServiceBrowser!
    private var connectedPeers: [MCPeerID] = []
    
    // Store the latest detection results for sharing via MCP
    private var latestDetectionResults: [RFObjectDetectionPrediction] = []
    
    // Store received detection data from peers
    private var receivedDetections: [[String: Any]] = []
    
    // Timer for automatic sending of detection data
    private var autoSendTimer: Timer?
    private var displayLink: CADisplayLink?
    private var targetTransmissionInterval: TimeInterval = 1.0 / 60.0 // 60fps
    private var lastScheduledTransmissionTime: CFTimeInterval = 0
    
    // Debug data for transmission statistics
    private var transmissionCount: Int = 0
    private var lastTransmissionTime: Date?
    private var transmissionRate: Double = 0
    private var lastTransmissionSize: Int = 0
    private var rawTransmissionData: String = ""
    private var jitterValues: [Double] = []
    private var averageJitter: Double = 0
    
    // UI Elements
    @IBOutlet weak var connectionStatusLabel: UILabel!
    private var debugDataLabel: UILabel!
    private var transmissionRateLabel: UILabel!
    private var jitterLabel: UILabel!
    private var toggleDebugButton: UIButton!
    
    // UI Visibility control
    private var debugViewVisible: Bool = true
    private var connectionButtons: [UIButton] = []
    
    // Triangulation properties
    private var triangulator = Triangulator()
    private var lastDetectionTime: TimeInterval = 0
    private var lastTriangulationTime: TimeInterval = 0
    private var triangulationLabel: UILabel!
    
    // Calibration properties
    private var calibrationMode: Bool = false
    private var calibrationButton: UIButton!
    private var calibrationInstructionsLabel: UILabel!
    private var calibrationStepLabel: UILabel!
    private var calibrationStep: Int = 0
    private var calibrationPoints: [CGPoint] = []
    
    // New status indicator
    private var statusIndicator: UIView?
    
    // Add calibration object constants
    private let calibrationObjectWidthInches: Double = 18.0
    private let calibrationObjectHeightInches: Double = 24.0
    private let inchesToMeters: Double = 0.0254 // Conversion factor from inches to meters
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view. //roboflow-mask-wearing-ios
        
        loadRoboflowModelWith(model: "line-detector-yytrt", version: 1 , threshold: 0.7, overlap: 0.2, maxObjects: 100.0)
        checkCameraAuthorization()
        
        // Setup MultipeerConnectivity
        setupMultipeerConnectivity()
        
        // Setup triangulation UI
        setupTriangulationUI()
        
        // Setup a timer to update the transmission rate display
        Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { [weak self] _ in
            self?.updateTransmissionRateDisplay()
        }
        
        // Initialize triangulator is already done in the property declaration
        
        // Ensure the local device calibrator is added with the correct device ID
        let deviceId = UIDevice.current.identifierForVendor?.uuidString ?? "unknown-device"
        print("Local device ID: \(deviceId)")
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        // Clean up MultipeerConnectivity
        autoSendTimer?.invalidate()
        autoSendTimer = nil
        
        // Clean up display link
        displayLink?.invalidate()
        displayLink = nil
        
        mcAdvertiser.stopAdvertisingPeer()
        mcBrowser.stopBrowsingForPeers()
        mcSession.disconnect()
    }
    
    //--------------------------
    //MARK: Camera Session
    //--------------------------
    
    func checkCameraAuthorization() {
        let authStatus = AVCaptureDevice.authorizationStatus(for: AVMediaType.video)
        
        if authStatus == AVAuthorizationStatus.denied {
            // Denied access to camera
            // Explain that we need camera access and how to change it.
            let dialog = UIAlertController(title: "Unable to access the Camera", message: "To enable access, go to Settings > Privacy > Camera and turn on Camera access for this app.", preferredStyle: UIAlertController.Style.alert)
            let okAction = UIAlertAction(title: "OK", style: UIAlertAction.Style.default, handler: nil)
            dialog.addAction(okAction)
            self.present(dialog, animated: true, completion: nil)
        } else if authStatus == AVAuthorizationStatus.notDetermined {
            // The user has not yet been presented with the option to grant access to the camera hardware.
            // Ask for it.
            AVCaptureDevice.requestAccess(for: AVMediaType.video, completionHandler: { [self] (granted) in
                if granted {
                    DispatchQueue.main.async { [self] in
                        //If we've been granted permission, start the camera session
                        setupAVCapture()
                    }
                }
            })
        } else {
            setupAVCapture()
        }
    }
    
    func setupAVCapture() {
        var deviceInput: AVCaptureDeviceInput!
        
        // Select a video device, make an input
        guard let videoDevice = AVCaptureDevice.DiscoverySession(deviceTypes: [.builtInWideAngleCamera], mediaType: .video, position: .front).devices.first else {
            let alert = UIAlertController(
                title: "No Camera Found",
                message: "You must run this app on a physical device with a camera.", preferredStyle: UIAlertController.Style.alert)
            alert.addAction(UIAlertAction(title: "OK", style: .cancel, handler: { (_) in
            }))
            self.present(alert, animated: true, completion: nil)
            return
        }
        do {
            deviceInput = try AVCaptureDeviceInput(device: videoDevice)
        } catch {
            print("Could not create video device input: \(error)")
            return
        }
        
        captureSession.beginConfiguration()
        captureSession.sessionPreset = .vga640x480
        
        // Add a video input
        guard captureSession.canAddInput(deviceInput) else {
            print("Could not add video device input to the session")
            captureSession.commitConfiguration()
            return
        }
        captureSession.addInput(deviceInput)
        
        if captureSession.canAddOutput(videoDataOutput) {
            captureSession.addOutput(videoDataOutput)
            // Add a video data output
            videoDataOutput.alwaysDiscardsLateVideoFrames = true
            videoDataOutput.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: Int(kCVPixelFormatType_420YpCbCr8BiPlanarFullRange)]
            videoDataOutput.setSampleBufferDelegate(self, queue: videoDataOutputQueue)
        } else {
            print("Could not add video data output to the session")
            captureSession.commitConfiguration()
            return
        }
        
        let captureConnection = videoDataOutput.connection(with: .video)
        // Always process the frames
        captureConnection?.isEnabled = true
        do {
            try  videoDevice.lockForConfiguration()
            let dimensions = CMVideoFormatDescriptionGetDimensions((videoDevice.activeFormat.formatDescription))
            bufferSize.width = CGFloat(dimensions.width)
            bufferSize.height = CGFloat(dimensions.height)
            videoDevice.unlockForConfiguration()
        } catch {
            print(error)
        }
        
        captureSession.commitConfiguration()
        
        DispatchQueue.main.async { [self] in
            previewLayer = AVCaptureVideoPreviewLayer(session: captureSession)
            previewLayer.videoGravity = AVLayerVideoGravity.resizeAspectFill
            rootLayer = previewView.layer
            previewLayer.frame = rootLayer.bounds
            rootLayer.addSublayer(previewLayer)
            
            setupLayers()
            updateLayerGeometry()
            startCaptureSession()
        }
    }
    
    @IBAction func changeCameraDirection(_ sender: Any) {
        switchCamera()
    }
    
    func stopCaptureSession() {
        self.captureSession.stopRunning()
        
        if let inputs = captureSession.inputs as? [AVCaptureDeviceInput] {
            for input in inputs {
                self.captureSession.removeInput(input)
            }
        }
    }
    
    func switchCamera() {
        captureSession.beginConfiguration()
        guard let currentInput = captureSession.inputs.first as? AVCaptureDeviceInput else {
            return
        }
        captureSession.removeInput(currentInput)
        
        guard let newCameraDevice = currentInput.device.position == .back ? getCamera(with: .front) : getCamera(with: .back) else { return
        }
        guard let newVideoInput = try? AVCaptureDeviceInput(device: newCameraDevice) else { return  }
        captureSession.addInput(newVideoInput)
        captureSession.commitConfiguration()
    }
    
    func getCamera(with position: AVCaptureDevice.Position) -> AVCaptureDevice? {
        let discoverySession = AVCaptureDevice.DiscoverySession(deviceTypes: [.builtInWideAngleCamera], mediaType: .video, position: position)
        return discoverySession.devices.first
    }
    
    
    func startCaptureSession() {
        DispatchQueue.global(qos: .background).async { [self] in
            captureSession.startRunning()
        }
    }
    
    func captureOutput(_ captureOutput: AVCaptureOutput, didDrop didDropSampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        print("frame dropped")
    }
    
    
    
    //--------------------------
    //MARK: Model Inference
    //--------------------------
    
    func loadRoboflowModelWith(model: String, version: Int,  threshold: Double, overlap: Double, maxObjects: Float) {
        rf.load(model: model, modelVersion: version) { [self] model, error, modelName, modelType in
            roboflowModel = model
            if error != nil {
                print(error?.localizedDescription as Any)
            } else {
                roboflowModel?.configure(threshold: threshold, overlap: overlap, maxObjects: maxObjects)
            }
        }
    }
    
    var start: DispatchTime!
    var end: DispatchTime!
    
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else {
            return
        }
        currentPixelBuffer = pixelBuffer
        
        let start: DispatchTime = .now()
        
        roboflowModel?.detect(pixelBuffer: pixelBuffer, completion: { detections, error in
            if error != nil {
                print("Error during detection: \(error!)")
            } else if let detectionResults = detections {
                // Print detections to the console
                print("Detected Objects (\(detectionResults.count) total):")
                
                for detection in detectionResults {
                    let detectionInfo = detection.vals()
                    print("---- Detection ----")
                    print("Class: \(detectionInfo["class"] ?? "Unknown")")
                    print("Confidence: \(detectionInfo["confidence"] ?? "N/A")")
                    print("X: \(detectionInfo["x"] ?? "N/A"), Y: \(detectionInfo["y"] ?? "N/A")")
                    print("Width: \(detectionInfo["width"] ?? "N/A"), Height: \(detectionInfo["height"] ?? "N/A")")
                    print("-------------------")
                }
                
                // Store the latest detection results for MCP sharing
                self.latestDetectionResults = detectionResults
                
                // Call function to draw bounding boxes and trajectory lines
                self.drawBoundingBoxesFrom(detections: detectionResults)
                
                // Calculate FPS
                DispatchQueue.main.async {
                    let duration = start.distance(to: .now())
                    let durationDouble = duration.toDouble()
                    let fps = round(1 / (durationDouble ?? 1.0))
                    self.fpsLabel.text = "\(fps) FPS"
                }
            }
        })
    }

//    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
//        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else {
//            return
//        }
//        currentPixelBuffer = pixelBuffer
//        
//        let start: DispatchTime = .now()
//        
//        roboflowModel?.detect(pixelBuffer: pixelBuffer, completion: { detections, error in
//            if error != nil {
//                print(error!)
//            } else {
//                let detectionResults: [RFObjectDetectionPrediction] = detections!
//                self.drawBoundingBoxesFrom(detections: detectionResults)
//                
//                //Caclulate and display the FPS of the ML inference
//                DispatchQueue.main.async { [self] in
//                    let duration = start.distance(to: .now())
//                    let durationDouble = duration.toDouble()
//                    var fps = 1 / durationDouble!
//                    fps = round(fps)
//                    fpsLabel.text = String(fps.description) + " FPS"
//                }
//            }
//        })
//    }
    
    //--------------------------
    //MARK: Bounding Boxes
    //--------------------------
    
    func setupLayers() {
        detectionOverlay = CALayer() // Container layer that has all the renderings of the bounding boxes
        detectionOverlay.name = "DetectionOverlay"
        detectionOverlay.bounds = CGRect(x: 0.0,
                                         y: 0.0,
                                         width: bufferSize.width,
                                         height: bufferSize.height)
        detectionOverlay.position = CGPoint(x: rootLayer.bounds.midX, y: rootLayer.bounds.midY)
        rootLayer.addSublayer(detectionOverlay)
    }
    
    func drawBoundingBoxesFrom(detections: [RFObjectDetectionPrediction]) {
        // Clear old bounding box layers
        CATransaction.begin()
        CATransaction.setValue(kCFBooleanTrue, forKey: kCATransactionDisableActions)
        detectionOverlay.sublayers = nil
        
        // --- Step 1: find the "pickleball" bounding box (if it exists) ---
        var ballCenter: CGPoint? = nil
        
        // Current time for this set of detections
        let currentTime = CACurrentMediaTime()
        lastDetectionTime = currentTime
        
        // Device identifier - ensure we use the same ID consistently throughout the app
        let deviceId = UIDevice.current.identifierForVendor?.uuidString ?? "unknown-device"
        
        for detection in detections {
            let detectionInfo = detection.vals()
            
            guard
                let detectedClass = detectionInfo["class"] as? String,
                let confidence = detectionInfo["confidence"] as? Double,
                let x = detectionInfo["x"] as? Float,
                let y = detectionInfo["y"] as? Float,
                let width = detectionInfo["width"] as? Float,
                let height = detectionInfo["height"] as? Float,
                let color = detectionInfo["color"] as? [Int]
            else {
                continue
            }
            
            // Filter for ball/pickleball detections only
            if !detectedClass.lowercased().contains("ball") {
                continue // Skip non-ball objects
            }
            
            // Convert to CG values
            let red: CGFloat = CGFloat(color[0]) / 255.0
            let green: CGFloat = CGFloat(color[1]) / 255.0
            let blue: CGFloat = CGFloat(color[2]) / 255.0
            let boundingBoxColor = UIColor(red: red, green: green, blue: blue, alpha: 0.25)
            
            let bounds = detectionOverlay.bounds
            let xs = bounds.width / bufferSize.width
            let ys = bounds.height / bufferSize.height
            
            let boundingBox = CGRect(
                x: CGFloat(x) * xs,
                y: CGFloat(y) * ys,
                width: CGFloat(width) * xs,
                height: CGFloat(height) * ys
            )
            
            // Draw the bounding box rectangle + text label
            drawBoundingBox(boundingBox: boundingBox,
                            color: boundingBoxColor,
                            detectedValue: detectedClass,
                            confidence: confidence)
            
            // Create a Detection2D object for triangulation
            let detection2D = Detection2D(
                x: Double(x),
                y: Double(y),
                width: Double(width),
                height: Double(height),
                confidence: confidence,
                timestamp: currentTime,
                deviceId: deviceId  // Use the consistent device ID
            )
            
            // Add to triangulator
            let performedTriangulation = triangulator.addDetection(detection: detection2D)
            
            if performedTriangulation {
                lastTriangulationTime = currentTime
                updateTriangulationDisplay()
            }
            
            // Send detection to peers
            sendDetectionToPeers(detection: detection2D)
            
            // Store the ball center for trajectory tracking
            let cX = boundingBox.midX
            let cY = boundingBox.midY
            ballCenter = CGPoint(x: cX, y: cY)
        }
        
        // --- Step 2: update the ball trajectory ---
        if let center = ballCenter {
            ballTrajectory.append(center)
            
            // We'll keep only last N points if you want (like a ring buffer).
            if ballTrajectory.count > 15 {
                ballTrajectory.removeFirst(ballTrajectory.count - 15)
            }
            
            // Detect bounce
            if ballTrajectory.detectBounce() {
                bounceFramesRemaining = 15  // e.g. display "bounce" for 15 frames
            }
        }
        
        // --- Step 3: draw the lines for the trajectory ---
        if ballTrajectory.count >= 2 {
            // We'll make a CAShapeLayer that draws lines between consecutive points
            let path = UIBezierPath()
            path.move(to: ballTrajectory[0])
            for i in 1..<ballTrajectory.count {
                path.addLine(to: ballTrajectory[i])
            }
            
            let trajectoryLayer = CAShapeLayer()
            trajectoryLayer.path = path.cgPath
            trajectoryLayer.strokeColor = UIColor.blue.cgColor
            trajectoryLayer.lineWidth = 3.0
            trajectoryLayer.fillColor = UIColor.clear.cgColor
            
            // We must rotate/scale just like bounding boxes
            // Instead, we can just add it as a sublayer of detectionOverlay
            // and rely on the same transform
            detectionOverlay.addSublayer(trajectoryLayer)
        }
        
        // --- Step 4: show "BOUNCE!" text if bounceFramesRemaining > 0 ---
        if bounceFramesRemaining > 0 {
            let textLayer = CATextLayer()
            textLayer.string = "BOUNCE!"
            textLayer.fontSize = 40
            textLayer.alignmentMode = .center
            textLayer.foregroundColor = UIColor.red.cgColor
            
            // Place it near the last known ball position if available
            if let lastPos = ballTrajectory.last {
                let size: CGFloat = 100
                textLayer.frame = CGRect(x: lastPos.x - size/2,
                                         y: lastPos.y - size/2,
                                         width: size,
                                         height: size)
            } else {
                // default top-left
                textLayer.frame = CGRect(x: 0, y: 0, width: 120, height: 50)
            }
            
            // The same transform as bounding boxes (rotate 90, mirror, etc.)
            textLayer.setAffineTransform(CGAffineTransform(rotationAngle: CGFloat(.pi / 2.0))
                .scaledBy(x: -1.0, y: -1.0))
            
            detectionOverlay.addSublayer(textLayer)
            
            // Decrement bounce frame count
            bounceFramesRemaining -= 1
        }
        
        // Draw any received detections from peers
        drawReceivedDetections()
        
        // Final commit
        CATransaction.commit()
        
        // Make sure you call `updateLayerGeometry()` afterwards
        self.updateLayerGeometry()
    }

//    func drawBoundingBoxesFrom(detections: [RFObjectDetectionPrediction]) {
//        CATransaction.begin()
//        CATransaction.setValue(kCFBooleanTrue, forKey: kCATransactionDisableActions)
//        detectionOverlay.sublayers = nil // Remove all the old recognized objects' bounding boxes from the UI
//        
//        //Extract the dictionary values of the predicted class
//        for detection in detections {
//            let detectionInfo = detection.vals()
//            guard let detectedValue = detectionInfo["class"] as? String else {
//                return
//            }
//            
//            guard let confidence = detectionInfo["confidence"] as? Double else {
//                return
//            }
//            
//            guard let x = detectionInfo["x"] as? Float else {
//                return
//            }
//            
//            guard let y = detectionInfo["y"] as? Float else {
//                return
//            }
//            
//            guard let width = detectionInfo["width"] as? Float else {
//                return
//            }
//            
//            guard let height = detectionInfo["height"] as? Float else {
//                return
//            }
//            
//            guard let color = detectionInfo["color"] as? [Int] else {
//                return
//            }
//            
//            //Calculate the shape, position, and color values of the detection bounding box
//            let red: Int = color[0]
//            let green: Int = color[1]
//            let blue: Int = color[2]
//            let boundingBoxColor = UIColor(red: CGFloat(red/255), green: CGFloat(green/255), blue: CGFloat(blue/255), alpha: 0.2)
//            let bounds = detectionOverlay.bounds
//            let xs = bounds.width/bufferSize.width
//            let ys = bounds.height/bufferSize.height
//            
//            //Create the CGRect for the bounding box, and draw it on the screen
//            let boundingBox: CGRect = CGRect(x: CGFloat(x)*xs, y: CGFloat(y)*ys, width: CGFloat(width)*xs, height: CGFloat(height)*ys)
//            drawBoundingBox(boundingBox: boundingBox, color: boundingBoxColor, detectedValue: detectedValue, confidence: confidence)
//        }
//        CATransaction.commit()
//    }
    
    //Create a bounding box and add it as a layer to the UI
    func drawBoundingBox(boundingBox: CGRect, color: UIColor, detectedValue: String, confidence: Double) {
        let shapeLayer = self.createRoundedRectLayerWithBounds(boundingBox, color: color)
        let textLayer = self.createTextSubLayerInBounds(boundingBox,
                                                        identifier: detectedValue,
                                                        confidence: VNConfidence(confidence))
        shapeLayer.addSublayer(textLayer)
        
        detectionOverlay.addSublayer(shapeLayer)
        self.updateLayerGeometry()
    }
    
    //Create a layer displaying the classification result and it's confidence
    func createTextSubLayerInBounds(_ bounds: CGRect, identifier: String, confidence: VNConfidence) -> CATextLayer {
        let textLayer = CATextLayer()
        textLayer.name = "Object Label"
        let confidenceString: String = ("Confidence: \(confidence)")
        
        let formattedString = NSMutableAttributedString(string: String(format: "\(identifier)\n\(confidenceString)"))
        let largeFont = UIFont(name: "Helvetica", size: 24.0)!
        
        formattedString.addAttributes([NSAttributedString.Key.font: largeFont], range: NSRange(location: 0, length: identifier.count))
        formattedString.addAttribute(NSAttributedString.Key.foregroundColor, value: UIColor.white, range: NSRange(location: 0, length: identifier.count + confidenceString.count + 1))
        
        textLayer.string = formattedString
        textLayer.bounds = CGRect(x: 0, y: 0, width: bounds.size.height - 10, height: bounds.size.width - 10)
        textLayer.position = CGPoint(x: bounds.midX, y: bounds.midY)
        textLayer.shadowOffset = CGSize(width: 2, height: 2)
        textLayer.foregroundColor = CGColor(colorSpace: CGColorSpaceCreateDeviceRGB(), components: [0.0, 0.0, 0.0, 1.0])
        textLayer.contentsScale = 2.0 // retina rendering
        
        // Rotate the layer into screen orientation and scale and mirror
        textLayer.setAffineTransform(CGAffineTransform(rotationAngle: CGFloat(.pi / 2.0)).scaledBy(x: -1.0, y: -1.0))
        return textLayer
    }
    
    //Creates the shape for bounding boxes to be displayed on the screen
    func createRoundedRectLayerWithBounds(_ bounds: CGRect, color: UIColor) -> CALayer {
        let shapeLayer = CALayer()
        shapeLayer.bounds = bounds
        shapeLayer.position = CGPoint(x: bounds.origin.x, y: bounds.origin.y)
        shapeLayer.name = "Found Object"
        
        var colorComponents = color.cgColor.components
        colorComponents?.removeLast()
        colorComponents?.append(0.4)
        shapeLayer.backgroundColor = CGColor(colorSpace: CGColorSpaceCreateDeviceRGB(), components: colorComponents!)
        shapeLayer.cornerRadius = 7
        return shapeLayer
    }
    
    //Update the position of the bounding-box overlay
    func updateLayerGeometry() {
        let bounds = rootLayer.bounds
        var scale: CGFloat
        
        let xScale: CGFloat = bounds.size.width / CGFloat(bufferSize.height)
        let yScale: CGFloat = bounds.size.height / CGFloat(bufferSize.width)
        
        scale = fmax(xScale, yScale)
        if scale.isInfinite {
            scale = 1.0
        }
        CATransaction.begin()
        CATransaction.setValue(kCFBooleanTrue, forKey: kCATransactionDisableActions)
        
        // Rotate the layer into screen orientation and scale and mirror
        detectionOverlay.setAffineTransform(CGAffineTransform(rotationAngle: CGFloat(.pi / 2.0)).scaledBy(x: scale, y: scale))
        // Center the layer
        detectionOverlay.position = CGPoint (x: bounds.midX, y: bounds.midY)
        
        CATransaction.commit()
    }
    
    //--------------------------
    //MARK: Image Uploading
    //--------------------------
    
    //Starts upload flow for if a user wants to upload the camera frame where an incorrect image classification occured
    @IBAction func uploadImage(_ sender: Any) {
        
        //Capture the current pixel buffer of the camera and convert it an image
        guard let pixelBuffer = currentPixelBuffer else {
            return
        }
        
        guard let capturedImage = UIImage(pixelBuffer: pixelBuffer) else {
            return
        }
        
        let rotatedImage = capturedImage.rotateImage(orientation: .down)
        
        let alert = UIAlertController(title: "Incorrect count?", message: "You've captured an image of this wrong count. Upload it to the open source dataset to improve this model.", preferredStyle: .alert)
        let imageView = UIImageView(frame: CGRect(x: 10, y: 100, width: 250, height: 230))
        imageView.image = rotatedImage
        alert.view.addSubview(imageView)
        let height = NSLayoutConstraint(item: alert.view!, attribute: .height, relatedBy: .equal, toItem: nil, attribute: .notAnAttribute, multiplier: 1, constant: 375)
        let width = NSLayoutConstraint(item: alert.view!, attribute: .width, relatedBy: .equal, toItem: nil, attribute: .notAnAttribute, multiplier: 1, constant: 250)
        alert.view.addConstraint(height)
        alert.view.addConstraint(width)
        
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel, handler: { (_) in
        }))
        alert.addAction(UIAlertAction(title: "Upload", style: .default, handler: { [self] (_) in
            //Upload the captured image to your dataset
            upload(image: rotatedImage)
        }))
        
        self.present(alert, animated: true, completion: nil)
    }
    
    //Uploads the incorrect classification frame
    func upload(image: UIImage) {
        let project = "roboflow-mask-wearing-ios"
        
        rf.uploadImage(image: image, project: project) { result in
            var title: String!
            var message: String!
            
            switch result {
            case .Success:
                title = "Success!"
                message = " Your image has been uploaded to the open source training dataset for model improvement."
            case .Duplicate:
                title = "Duplicate"
                message = "You attempted to upload a duplicate image."
            case .Error:
                title = "Error"
                message = "An error occured while uploading your image."
            @unknown default:
                return
            }
            
            DispatchQueue.main.async {
                let alert = UIAlertController(title: title, message: message, preferredStyle: UIAlertController.Style.alert)
                alert.addAction(UIAlertAction(title: "OK", style: .cancel, handler: { (_) in
                }))
                self.present(alert, animated: true, completion: nil)
            }
        }
    }
    
    //--------------------------
    //MARK: MultipeerConnectivity
    //--------------------------
    
    func setupMultipeerConnectivity() {
        // Initialize with device name
        myPeerId = MCPeerID(displayName: UIDevice.current.name)
        
        // Initialize session
        mcSession = MCSession(peer: myPeerId, securityIdentity: nil, encryptionPreference: .required)
        mcSession.delegate = self
        
        // Initialize advertiser (to be discovered)
        mcAdvertiser = MCNearbyServiceAdvertiser(peer: myPeerId, discoveryInfo: nil, serviceType: serviceType)
        mcAdvertiser.delegate = self
        
        // Initialize browser (to discover others)
        mcBrowser = MCNearbyServiceBrowser(peer: myPeerId, serviceType: serviceType)
        mcBrowser.delegate = self
        
        // Add toggle debug button
        toggleDebugButton = UIButton(frame: CGRect(x: view.bounds.width - 90, y: 50, width: 80, height: 40))
        toggleDebugButton.setTitle("Hide UI", for: .normal)
        toggleDebugButton.backgroundColor = .systemBlue
        toggleDebugButton.layer.cornerRadius = 8
        toggleDebugButton.titleLabel?.font = UIFont.systemFont(ofSize: 14, weight: .medium)
        toggleDebugButton.addTarget(self, action: #selector(toggleDebugView), for: .touchUpInside)
        view.addSubview(toggleDebugButton)
        
        // Add calibration button
        calibrationButton = UIButton(frame: CGRect(x: 20, y: view.bounds.height - 210, width: view.bounds.width - 40, height: 50))
        calibrationButton.setTitle("Start Calibration", for: .normal)
        calibrationButton.backgroundColor = .systemPurple
        calibrationButton.layer.cornerRadius = 8
        calibrationButton.addTarget(self, action: #selector(toggleCalibrationMode), for: .touchUpInside)
        view.addSubview(calibrationButton)
        
        // Add connection status label if it doesn't exist yet
        if connectionStatusLabel == nil {
            let statusLabel = UILabel(frame: CGRect(x: 20, y: 40, width: view.bounds.width - 40, height: 30))
            statusLabel.textAlignment = .center
            statusLabel.textColor = .white
            statusLabel.backgroundColor = UIColor.black.withAlphaComponent(0.5)
            statusLabel.layer.cornerRadius = 8
            statusLabel.clipsToBounds = true
            statusLabel.text = "Not Connected"
            view.addSubview(statusLabel)
            connectionStatusLabel = statusLabel
        }
        
        // Add transmission rate label
        transmissionRateLabel = UILabel(frame: CGRect(x: 20, y: 80, width: view.bounds.width - 40, height: 30))
        transmissionRateLabel.textAlignment = .center
        transmissionRateLabel.textColor = .white
        transmissionRateLabel.backgroundColor = UIColor.black.withAlphaComponent(0.5)
        transmissionRateLabel.layer.cornerRadius = 8
        transmissionRateLabel.clipsToBounds = true
        transmissionRateLabel.text = "Transmission Rate: 0 Hz"
        view.addSubview(transmissionRateLabel)
        
        // Add jitter label
        jitterLabel = UILabel(frame: CGRect(x: 20, y: 120, width: view.bounds.width - 40, height: 30))
        jitterLabel.textAlignment = .center
        jitterLabel.textColor = .white
        jitterLabel.backgroundColor = UIColor.black.withAlphaComponent(0.5)
        jitterLabel.layer.cornerRadius = 8
        jitterLabel.clipsToBounds = true
        jitterLabel.text = "Jitter: 0 ms"
        view.addSubview(jitterLabel)
        
        // Add debug data label
        debugDataLabel = UILabel(frame: CGRect(x: 20, y: 160, width: view.bounds.width - 40, height: 60))
        debugDataLabel.textAlignment = .left
        debugDataLabel.textColor = .white
        debugDataLabel.backgroundColor = UIColor.black.withAlphaComponent(0.5)
        debugDataLabel.layer.cornerRadius = 8
        debugDataLabel.clipsToBounds = true
        debugDataLabel.numberOfLines = 3
        debugDataLabel.font = UIFont.systemFont(ofSize: 12)
        debugDataLabel.text = "Data: None"
        view.addSubview(debugDataLabel)
        
        // Add connection buttons
        addConnectionButtons()
    }
    
    func addConnectionButtons() {
        // Clear any existing buttons
        connectionButtons.forEach { $0.removeFromSuperview() }
        connectionButtons.removeAll()
        
        // Host button
        let hostButton = UIButton(frame: CGRect(x: 20, y: view.bounds.height - 150, width: view.bounds.width/2 - 30, height: 50))
        hostButton.setTitle("Host", for: .normal)
        hostButton.backgroundColor = .systemBlue
        hostButton.layer.cornerRadius = 8
        hostButton.addTarget(self, action: #selector(startHosting), for: .touchUpInside)
        view.addSubview(hostButton)
        connectionButtons.append(hostButton)
        
        // Join button
        let joinButton = UIButton(frame: CGRect(x: view.bounds.width/2 + 10, y: view.bounds.height - 150, width: view.bounds.width/2 - 30, height: 50))
        joinButton.setTitle("Join", for: .normal)
        joinButton.backgroundColor = .systemGreen
        joinButton.layer.cornerRadius = 8
        joinButton.addTarget(self, action: #selector(joinSession), for: .touchUpInside)
        view.addSubview(joinButton)
        connectionButtons.append(joinButton)
        
        // Send Data button
        let sendButton = UIButton(frame: CGRect(x: 20, y: view.bounds.height - 90, width: view.bounds.width - 40, height: 50))
        sendButton.setTitle("Send Detection Data", for: .normal)
        sendButton.backgroundColor = .systemOrange
        sendButton.layer.cornerRadius = 8
        sendButton.addTarget(self, action: #selector(sendDetectionData), for: .touchUpInside)
        view.addSubview(sendButton)
        connectionButtons.append(sendButton)
        
        // Auto-send toggle button
        let autoSendButton = UIButton(frame: CGRect(x: 20, y: view.bounds.height - 30, width: view.bounds.width - 40, height: 30))
        autoSendButton.setTitle("Toggle Auto-Send", for: .normal)
        autoSendButton.backgroundColor = .systemPurple
        autoSendButton.layer.cornerRadius = 8
        autoSendButton.addTarget(self, action: #selector(toggleAutoSend), for: .touchUpInside)
        view.addSubview(autoSendButton)
        connectionButtons.append(autoSendButton)
        
        // Add calibration button to connection buttons array
        connectionButtons.append(calibrationButton)
    }
    
    @objc func startHosting() {
        mcAdvertiser.startAdvertisingPeer()
        updateConnectionStatus(status: "Hosting - Waiting for connections")
    }
    
    @objc func joinSession() {
        // Present browser view controller
        let mcBrowserVC = MCBrowserViewController(serviceType: serviceType, session: mcSession)
        mcBrowserVC.delegate = self as MCBrowserViewControllerDelegate
        present(mcBrowserVC, animated: true)
    }
    
    @objc func sendDetectionData() {
        // Only send if we have peers connected
        guard !mcSession.connectedPeers.isEmpty else {
            return
        }
        
        // Create a simple message with the latest detection data
        if let detections = getLatestDetections() {
            do {
                // Use compact JSON encoding for minimal size
                let jsonOptions: JSONSerialization.WritingOptions = [.fragmentsAllowed, .sortedKeys, .withoutEscapingSlashes]
                let data = try JSONSerialization.data(withJSONObject: detections, options: jsonOptions)
                
                // Use unreliable mode for lowest latency
                try mcSession.send(data, toPeers: mcSession.connectedPeers, with: .unreliable)
                
                // Update transmission statistics
                transmissionCount += 1
                lastTransmissionSize = data.count
                
                // Calculate transmission rate
                let now = Date()
                if let lastTime = lastTransmissionTime {
                    let timeDiff = now.timeIntervalSince(lastTime)
                    if timeDiff > 0 {
                        // Calculate a moving average of the transmission rate
                        let instantRate = 1.0 / timeDiff
                        transmissionRate = 0.9 * transmissionRate + 0.1 * instantRate // More weight on history for stability
                    }
                }
                lastTransmissionTime = now
                
                // Update the raw data display (limit to a reasonable size)
                if let jsonString = String(data: data, encoding: .utf8) {
                    rawTransmissionData = String(jsonString.prefix(100)) + (jsonString.count > 100 ? "..." : "")
                }
                
                // Update the UI (but not on every frame to avoid UI bottlenecks)
                if transmissionCount % 10 == 0 {
                    updateTransmissionRateDisplay()
                }
                
            } catch {
                print("Error sending data: \(error.localizedDescription)")
            }
        }
    }
    
    func updateTransmissionRateDisplay() {
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            
            // Update transmission rate label
            self.transmissionRateLabel.text = String(format: "Rate: %.1f Hz | Size: %d bytes", self.transmissionRate, self.lastTransmissionSize)
            
            // Update jitter label
            self.jitterLabel.text = String(format: "Jitter: %.2f ms (target: 16.67ms)", self.averageJitter)
            
            // Update debug data label
            self.debugDataLabel.text = "Count: \(self.transmissionCount)\nLast: \(self.lastTransmissionTime?.description.suffix(8) ?? "none")\nData: \(self.rawTransmissionData)"
        }
    }
    
    @objc func toggleAutoSend() {
        if displayLink == nil {
            // Start auto-sending at 60fps using CADisplayLink for precise timing
            displayLink = CADisplayLink(target: self, selector: #selector(sendDataOnDisplayLink))
            displayLink?.preferredFramesPerSecond = 60
            displayLink?.add(to: .main, forMode: .common)
            
            // Reset statistics
            transmissionCount = 0
            lastTransmissionTime = nil
            transmissionRate = 0
            jitterValues = []
            averageJitter = 0
            lastScheduledTransmissionTime = 0
            
            updateConnectionStatus(status: "Auto-send enabled (60Hz)")
        } else {
            // Stop auto-sending
            displayLink?.invalidate()
            displayLink = nil
            
            // Reset transmission statistics
            transmissionCount = 0
            lastTransmissionTime = nil
            transmissionRate = 0
            jitterValues = []
            averageJitter = 0
            
            updateConnectionStatus(status: "Auto-send disabled")
            updateTransmissionRateDisplay()
        }
    }
    
    @objc func sendDataOnDisplayLink(displayLink: CADisplayLink) {
        // Calculate time since last scheduled transmission
        let currentTime = displayLink.timestamp
        
        // On first run, just initialize the time
        if lastScheduledTransmissionTime == 0 {
            lastScheduledTransmissionTime = currentTime
            return
        }
        
        // Calculate actual interval and jitter
        let actualInterval = currentTime - lastScheduledTransmissionTime
        let jitter = abs(actualInterval - targetTransmissionInterval) * 1000 // in milliseconds
        
        // Update jitter statistics (keep last 60 values - 1 second worth)
        jitterValues.append(jitter)
        if jitterValues.count > 60 {
            jitterValues.removeFirst()
        }
        
        // Calculate average jitter
        if !jitterValues.isEmpty {
            averageJitter = jitterValues.reduce(0, +) / Double(jitterValues.count)
        }
        
        // Update the last scheduled time
        lastScheduledTransmissionTime = currentTime
        
        // Send the data
        sendDetectionData()
    }
    
    func getLatestDetections() -> [[String: Any]]? {
        // Always create a data packet with timestamp, even if no detections
        var packet: [String: Any] = [
            "ts": Date().timeIntervalSince1970, // Timestamp for jitter calculation
            "id": transmissionCount // Sequence number for packet loss detection
        ]
        
        // Convert the latest detection results to a serializable format with minimal data
        var serializableDetections: [[String: Any]] = []
        
        if !latestDetectionResults.isEmpty {
            for detection in latestDetectionResults {
                // Only include essential data to minimize payload size
                let detectionInfo = detection.vals()
                var minimalDetection: [String: Any] = [:]
                
                // Include only the essential fields
                if let cls = detectionInfo["class"] as? String {
                    minimalDetection["c"] = cls
                }
                if let confidence = detectionInfo["confidence"] as? Double {
                    // Round confidence to 2 decimal places to reduce data size
                    minimalDetection["cf"] = round(confidence * 100) / 100
                }
                if let x = detectionInfo["x"] as? Float {
                    minimalDetection["x"] = Int(x)
                }
                if let y = detectionInfo["y"] as? Float {
                    minimalDetection["y"] = Int(y)
                }
                if let width = detectionInfo["width"] as? Float {
                    minimalDetection["w"] = Int(width)
                }
                if let height = detectionInfo["height"] as? Float {
                    minimalDetection["h"] = Int(height)
                }
                
                serializableDetections.append(minimalDetection)
            }
            
            // Add detections to the packet
            packet["d"] = serializableDetections
        } else {
            // No detections available, send empty array
            packet["d"] = []
        }
        
        // Return the packet as a single-element array to maintain compatibility
        return [packet]
    }
    
    func updateConnectionStatus(status: String) {
        DispatchQueue.main.async { [weak self] in
            self?.connectionStatusLabel.text = status
        }
    }
    
    func showAlert(title: String, message: String) {
        let alert = UIAlertController(title: title, message: message, preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: "OK", style: .default))
        present(alert, animated: true)
    }
    
    // MARK: - MCSessionDelegate
    
    func session(_ session: MCSession, peer peerID: MCPeerID, didChange state: MCSessionState) {
        switch state {
        case .connected:
            print("Connected to: \(peerID.displayName)")
            updateConnectionStatus(status: "Connected to: \(peerID.displayName)")
            updateConnectionIndicator(connected: true)
        case .connecting:
            print("Connecting to: \(peerID.displayName)")
            updateConnectionStatus(status: "Connecting to: \(peerID.displayName)")
        case .notConnected:
            print("Not connected to: \(peerID.displayName)")
            updateConnectionStatus(status: "Disconnected from: \(peerID.displayName)")
            updateConnectionIndicator(connected: false)
        @unknown default:
            print("Unknown connection state: \(peerID.displayName)")
        }
    }
    
    // Helper method to update the connection indicator
    private func updateConnectionIndicator(connected: Bool) {
        DispatchQueue.main.async { [weak self] in
            guard let self = self, !self.debugViewVisible else { return }
            
            // Find and update the indicator if it exists
            if let indicator = self.view.viewWithTag(999) {
                indicator.backgroundColor = connected ? .green : .red
            }
        }
    }
    
    func session(_ session: MCSession, didReceive data: Data, fromPeer peerID: MCPeerID) {
        // Process received data
        do {
            if let json = try JSONSerialization.jsonObject(with: data, options: []) as? [String: Any],
               let type = json["type"] as? String {
                
                print("Received data of type: \(type) from peer: \(peerID.displayName)")
                
                switch type {
                case "detection":
                    if let detectionData = json["data"] as? [String: Any],
                       let detection = Detection2D.fromDictionary(detectionData) {
                        
                        print("Received detection from device: \(detection.deviceId)")
                        
                        // Process the detection for triangulation
                        let performedTriangulation = triangulator.addDetection(detection: detection)
                        
                        if performedTriangulation {
                            DispatchQueue.main.async {
                                self.lastTriangulationTime = CACurrentMediaTime()
                                self.updateTriangulationDisplay()
                            }
                        }
                    }
                    
                case "calibration":
                    if let deviceId = json["device_id"] as? String,
                       let intrinsicsDict = json["intrinsics"] as? [String: Any],
                       let extrinsicsDict = json["extrinsics"] as? [String: Any] {
                        
                        print("Received calibration data for device: \(deviceId)")
                        
                        // Create calibrator from received data
                        let calibrator = CameraCalibrator()
                        
                        // Set intrinsics and extrinsics from dictionaries
                        if let intrinsics = CameraIntrinsics.fromDictionary(intrinsicsDict) {
                            calibrator.setIntrinsics(intrinsics)
                        }
                        
                        if let extrinsics = CameraExtrinsics.fromDictionary(extrinsicsDict) {
                            calibrator.setExtrinsics(extrinsics)
                        }
                        
                        // Add to triangulator
                        triangulator.addCalibrator(calibrator: calibrator, deviceId: deviceId)
                        print("Added calibrator for device: \(deviceId)")
                        print("Available calibrators: \(triangulator.getAvailableCalibrators())")
                        
                        // Update UI
                        DispatchQueue.main.async {
                            self.updateConnectionStatus(status: "Received calibration from: \(peerID.displayName)")
                        }
                    }
                    
                default:
                    // Handle legacy data format
                    processReceivedData(data, fromPeer: peerID)
                }
            } else {
                // Handle legacy data format
                processReceivedData(data, fromPeer: peerID)
            }
        } catch {
            print("Error processing received data: \(error.localizedDescription)")
            processReceivedData(data, fromPeer: peerID)
        }
    }
    
    // Helper method to process received data in the original format
    private func processReceivedData(_ data: Data, fromPeer peerID: MCPeerID) {
        do {
            if let receivedPackets = try JSONSerialization.jsonObject(with: data) as? [[String: Any]],
               let packet = receivedPackets.first {
                
                // Extract timestamp and calculate network latency
                var latency: Double = 0
                if let timestamp = packet["ts"] as? Double {
                    latency = Date().timeIntervalSince1970 - timestamp
                }
                
                // Extract sequence number to detect packet loss
                let sequenceNumber = packet["id"] as? Int ?? 0
                
                // Extract detections
                if let detections = packet["d"] as? [[String: Any]] {
                    // Store the received detections
                    self.receivedDetections = detections
                    
                    // Update the UI to show we received data
                    DispatchQueue.main.async {
                        // Show data size and content
                        let dataSize = data.count
                        var dataPreview = "No data"
                        if let jsonString = String(data: data, encoding: .utf8) {
                            dataPreview = String(jsonString.prefix(100)) + (jsonString.count > 100 ? "..." : "")
                        }
                        
                        self.updateConnectionStatus(status: "Received #\(sequenceNumber) | Latency: \(String(format: "%.1f", latency * 1000))ms")
                        self.debugDataLabel.text = "Received: \(dataSize) bytes\nFrom: \(peerID.displayName)\nData: \(dataPreview)"
                        
                        // Trigger redraw to show the received detections
                        self.drawReceivedDetections()
                    }
                }
            }
        } catch {
            print("Error processing received data: \(error.localizedDescription)")
        }
    }
    
    func session(_ session: MCSession, didReceive stream: InputStream, withName streamName: String, fromPeer peerID: MCPeerID) {
        // Not implemented for this basic example
    }
    
    func session(_ session: MCSession, didStartReceivingResourceWithName resourceName: String, fromPeer peerID: MCPeerID, with progress: Progress) {
        // Not implemented for this basic example
    }
    
    func session(_ session: MCSession, didFinishReceivingResourceWithName resourceName: String, fromPeer peerID: MCPeerID, at localURL: URL?, withError error: Error?) {
        // Not implemented for this basic example
    }
    
    // MARK: - MCNearbyServiceAdvertiserDelegate
    
    func advertiser(_ advertiser: MCNearbyServiceAdvertiser, didReceiveInvitationFromPeer peerID: MCPeerID, withContext context: Data?, invitationHandler: @escaping (Bool, MCSession?) -> Void) {
        // Auto-accept the invitation
        let alert = UIAlertController(title: "Invitation", message: "Would you like to connect with \(peerID.displayName)?", preferredStyle: .alert)
        
        alert.addAction(UIAlertAction(title: "Accept", style: .default, handler: { _ in
            invitationHandler(true, self.mcSession)
        }))
        
        alert.addAction(UIAlertAction(title: "Decline", style: .cancel, handler: { _ in
            invitationHandler(false, nil)
        }))
        
        DispatchQueue.main.async {
            self.present(alert, animated: true)
        }
    }
    
    // MARK: - MCNearbyServiceBrowserDelegate
    
    func browser(_ browser: MCNearbyServiceBrowser, foundPeer peerID: MCPeerID, withDiscoveryInfo info: [String: String]?) {
        print("Found peer: \(peerID.displayName)")
        // You could automatically invite the peer here if desired
        // browser.invitePeer(peerID, to: mcSession, withContext: nil, timeout: 10)
    }
    
    func browser(_ browser: MCNearbyServiceBrowser, lostPeer peerID: MCPeerID) {
        print("Lost peer: \(peerID.displayName)")
    }
    
    // Draw detections received from peers with a different color
    func drawReceivedDetections() {
        // Use a different color for received detections to distinguish them
        let peerColor = UIColor(red: 1.0, green: 0.5, blue: 0.0, alpha: 0.4) // Orange
        
        for detection in receivedDetections {
            // Handle both the minimized and original data formats
            let detectedClass = detection["c"] as? String ?? detection["class"] as? String
            let confidence = detection["cf"] as? Double ?? detection["confidence"] as? Double
            
            // Handle both Int and Float types for coordinates
            var x: Float?
            if let xInt = detection["x"] as? Int {
                x = Float(xInt)
            } else {
                x = detection["x"] as? Float
            }
            
            var y: Float?
            if let yInt = detection["y"] as? Int {
                y = Float(yInt)
            } else {
                y = detection["y"] as? Float
            }
            
            var width: Float?
            if let wInt = detection["w"] as? Int {
                width = Float(wInt)
            } else if let wInt = detection["width"] as? Int {
                width = Float(wInt)
            } else {
                width = detection["width"] as? Float
            }
            
            var height: Float?
            if let hInt = detection["h"] as? Int {
                height = Float(hInt)
            } else if let hInt = detection["height"] as? Int {
                height = Float(hInt)
            } else {
                height = detection["height"] as? Float
            }
            
            // Make sure we have all required values
            guard let cls = detectedClass,
                  let conf = confidence,
                  let xVal = x,
                  let yVal = y,
                  let widthVal = width,
                  let heightVal = height else {
                continue
            }
            
            let bounds = detectionOverlay.bounds
            let xs = bounds.width / bufferSize.width
            let ys = bounds.height / bufferSize.height
            
            let boundingBox = CGRect(
                x: CGFloat(xVal) * xs,
                y: CGFloat(yVal) * ys,
                width: CGFloat(widthVal) * xs,
                height: CGFloat(heightVal) * ys
            )
            
            // Draw the bounding box with a different color to indicate it's from a peer
            drawBoundingBox(boundingBox: boundingBox,
                            color: peerColor,
                            detectedValue: "\(cls) (peer)",
                            confidence: conf)
        }
    }
    
    // MARK: - MCBrowserViewControllerDelegate
    
    func browserViewControllerDidFinish(_ browserViewController: MCBrowserViewController) {
        // Handle the completion of the browser view controller
        dismiss(animated: true, completion: nil)
    }
    
    func browserViewControllerWasCancelled(_ browserViewController: MCBrowserViewController) {
        // Handle the cancellation of the browser view controller
        dismiss(animated: true, completion: nil)
    }
    
    @objc func toggleDebugView() {
        debugViewVisible = !debugViewVisible
        
        // Update button title
        if debugViewVisible {
            toggleDebugButton.setTitle("Hide UI", for: .normal)
        } else {
            toggleDebugButton.setTitle("Show UI", for: .normal)
        }
        
        // Toggle visibility of debug elements
        connectionStatusLabel.isHidden = !debugViewVisible
        transmissionRateLabel.isHidden = !debugViewVisible
        jitterLabel.isHidden = !debugViewVisible
        debugDataLabel.isHidden = !debugViewVisible
        triangulationLabel.isHidden = !debugViewVisible
        
        // Toggle visibility of connection buttons
        connectionButtons.forEach { $0.isHidden = !debugViewVisible }
        
        // Toggle visibility of calibration button (but not the calibration UI if in calibration mode)
        calibrationButton.isHidden = !debugViewVisible
        if !calibrationMode {
            calibrationInstructionsLabel.isHidden = true
            calibrationStepLabel.isHidden = true
        }
        
        // When debug view is hidden, show a small connection status indicator
        if !debugViewVisible {
            // Create a small indicator next to the toggle button
            statusIndicator = UIView(frame: CGRect(x: view.bounds.width - 100, y: 65, width: 10, height: 10))
            statusIndicator?.layer.cornerRadius = 5
            statusIndicator?.tag = 999 // Use tag to find and remove it later
            
            // Set color based on connection status
            if mcSession.connectedPeers.isEmpty {
                statusIndicator?.backgroundColor = .red // Not connected
            } else {
                statusIndicator?.backgroundColor = .green // Connected
            }
            
            view.addSubview(statusIndicator!)
        } else {
            // Remove the indicator when debug view is shown
            statusIndicator?.removeFromSuperview()
            statusIndicator = nil
        }
    }
    
    private func setupTriangulationUI() {
        // Create triangulation position label
        triangulationLabel = UILabel()
        triangulationLabel.frame = CGRect(x: 20, y: 200, width: view.bounds.width - 40, height: 60)
        triangulationLabel.textColor = .white
        triangulationLabel.backgroundColor = UIColor.black.withAlphaComponent(0.5)
        triangulationLabel.numberOfLines = 3
        triangulationLabel.textAlignment = .left
        triangulationLabel.font = UIFont.systemFont(ofSize: 12)
        triangulationLabel.layer.cornerRadius = 5
        triangulationLabel.clipsToBounds = true
        triangulationLabel.text = "Waiting for triangulation data..."
        triangulationLabel.isHidden = !debugViewVisible
        view.addSubview(triangulationLabel)
        
        // Add calibration instructions label
        calibrationInstructionsLabel = UILabel(frame: CGRect(x: 20, y: 270, width: view.bounds.width - 40, height: 80))
        calibrationInstructionsLabel.textColor = .white
        calibrationInstructionsLabel.backgroundColor = UIColor.black.withAlphaComponent(0.5)
        calibrationInstructionsLabel.numberOfLines = 4
        calibrationInstructionsLabel.textAlignment = .center
        calibrationInstructionsLabel.font = UIFont.systemFont(ofSize: 14)
        calibrationInstructionsLabel.layer.cornerRadius = 5
        calibrationInstructionsLabel.clipsToBounds = true
        calibrationInstructionsLabel.text = "Calibration will help establish the relative positions of the devices."
        calibrationInstructionsLabel.isHidden = true
        view.addSubview(calibrationInstructionsLabel)
        
        // Add calibration step label
        calibrationStepLabel = UILabel(frame: CGRect(x: 20, y: 360, width: view.bounds.width - 40, height: 40))
        calibrationStepLabel.textColor = .white
        calibrationStepLabel.backgroundColor = UIColor.systemBlue.withAlphaComponent(0.7)
        calibrationStepLabel.numberOfLines = 2
        calibrationStepLabel.textAlignment = .center
        calibrationStepLabel.font = UIFont.boldSystemFont(ofSize: 16)
        calibrationStepLabel.layer.cornerRadius = 5
        calibrationStepLabel.clipsToBounds = true
        calibrationStepLabel.text = "Step 1: Place calibration object"
        calibrationStepLabel.isHidden = true
        view.addSubview(calibrationStepLabel)
    }
    
    // MARK: - Calibration Methods
    
    @objc func toggleCalibrationMode() {
        calibrationMode = !calibrationMode
        
        if calibrationMode {
            // Start calibration
            calibrationButton.setTitle("Cancel Calibration", for: .normal)
            calibrationButton.backgroundColor = .systemRed
            
            // Show calibration UI
            calibrationInstructionsLabel.isHidden = false
            calibrationStepLabel.isHidden = false
            
            // Reset calibration state
            calibrationStep = 1
            calibrationPoints.removeAll()
            
            // Update instructions
            updateCalibrationInstructions()
            
            // Add tap gesture recognizer for calibration points
            let tapGesture = UITapGestureRecognizer(target: self, action: #selector(handleCalibrationTap(_:)))
            view.addGestureRecognizer(tapGesture)
            
        } else {
            // End calibration
            calibrationButton.setTitle("Start Calibration", for: .normal)
            calibrationButton.backgroundColor = .systemPurple
            
            // Hide calibration UI
            calibrationInstructionsLabel.isHidden = true
            calibrationStepLabel.isHidden = true
            
            // Remove tap gesture recognizer
            for recognizer in view.gestureRecognizers ?? [] {
                if let tapGesture = recognizer as? UITapGestureRecognizer {
                    // Simply remove all tap gesture recognizers since we only add one for calibration
                    view.removeGestureRecognizer(tapGesture)
                }
            }
        }
    }
    
    private func updateCalibrationInstructions() {
        switch calibrationStep {
        case 1:
            calibrationStepLabel.text = "Step 1: Tap on the top-left corner of the calibration object"
            calibrationInstructionsLabel.text = "Place an 18\" × 24\" calibration object (e.g., a poster board) in view of both cameras."
        case 2:
            calibrationStepLabel.text = "Step 2: Tap on the top-right corner of the calibration object"
            calibrationInstructionsLabel.text = "Make sure the 18\" × 24\" object is clearly visible and not moving."
        case 3:
            calibrationStepLabel.text = "Step 3: Tap on the bottom-right corner of the calibration object"
            calibrationInstructionsLabel.text = "Keep the device steady during calibration."
        case 4:
            calibrationStepLabel.text = "Step 4: Tap on the bottom-left corner of the calibration object"
            calibrationInstructionsLabel.text = "Almost done! This is the last corner to tap."
        case 5:
            calibrationStepLabel.text = "Calibration Complete!"
            calibrationInstructionsLabel.text = "Calibration data has been saved and shared with the other device."
            
            // Perform the actual calibration
            performCalibration()
            
            // Auto-exit calibration mode after a delay
            DispatchQueue.main.asyncAfter(deadline: .now() + 2.0) {
                self.toggleCalibrationMode()
            }
        default:
            break
        }
    }
    
    @objc func handleCalibrationTap(_ gesture: UITapGestureRecognizer) {
        guard calibrationMode && calibrationStep <= 4 else { return }
        
        // Get the tap location in the view first, then convert to the preview layer's coordinate space
        let locationInView = gesture.location(in: view)
        let locationInPreviewLayer = previewLayer.convert(locationInView, from: view.layer)
        
        // Store the calibration point
        calibrationPoints.append(locationInPreviewLayer)
        
        // Draw a marker at the tapped location (in view coordinates)
        drawCalibrationMarker(at: locationInView)
        
        // Move to the next step
        calibrationStep += 1
        updateCalibrationInstructions()
    }
    
    private func drawCalibrationMarker(at point: CGPoint) {
        // Create a marker view
        let markerSize: CGFloat = 20
        let markerView = UIView(frame: CGRect(x: point.x - markerSize/2, y: point.y - markerSize/2, width: markerSize, height: markerSize))
        markerView.backgroundColor = .systemYellow
        markerView.layer.cornerRadius = markerSize/2
        markerView.alpha = 0.7
        markerView.tag = 1000 + calibrationPoints.count // Use tag to identify markers
        
        // Add a label with the point number
        let label = UILabel(frame: markerView.bounds)
        label.text = "\(calibrationPoints.count)"
        label.textAlignment = .center
        label.textColor = .black
        label.font = UIFont.boldSystemFont(ofSize: 12)
        markerView.addSubview(label)
        
        // Add to the view
        view.addSubview(markerView)
        
        // Animate the marker to confirm the tap
        UIView.animate(withDuration: 0.2, animations: {
            markerView.transform = CGAffineTransform(scaleX: 1.5, y: 1.5)
        }) { _ in
            UIView.animate(withDuration: 0.2) {
                markerView.transform = .identity
            }
        }
    }
    
    private func performCalibration() {
        guard calibrationPoints.count == 4 else { return }
        
        // Convert screen points to normalized image coordinates
        let imagePoints = calibrationPoints.map { point -> CGPoint in
            // Convert from screen coordinates to normalized image coordinates (0-1)
            let normalizedX = point.x / previewLayer.bounds.width
            let normalizedY = point.y / previewLayer.bounds.height
            return CGPoint(x: normalizedX, y: normalizedY)
        }
        
        // Calculate the real-world dimensions of the calibration object in meters
        let objectWidthMeters = calibrationObjectWidthInches * inchesToMeters
        let objectHeightMeters = calibrationObjectHeightInches * inchesToMeters
        
        // Create a device ID - ensure we use the same ID consistently throughout the app
        let deviceId = UIDevice.current.identifierForVendor?.uuidString ?? "unknown-device"
        print("Performing calibration for device ID: \(deviceId)")
        
        // Create a calibrator with default intrinsics
        let calibrator = CameraCalibrator()
        
        // Calculate the scale factor based on the known object size
        // This is a simplified approach - in a production app, you would use more sophisticated algorithms
        let pixelWidth = hypot(
            calibrationPoints[1].x - calibrationPoints[0].x,
            calibrationPoints[1].y - calibrationPoints[0].y
        )
        let pixelHeight = hypot(
            calibrationPoints[3].x - calibrationPoints[0].x,
            calibrationPoints[3].y - calibrationPoints[0].y
        )
        
        // Calculate pixels per meter for width and height
        let pixelsPerMeterWidth = pixelWidth / objectWidthMeters
        let pixelsPerMeterHeight = pixelHeight / objectHeightMeters
        
        // Use the average for a more robust estimate
        let pixelsPerMeter = (pixelsPerMeterWidth + pixelsPerMeterHeight) / 2.0
        
        // Calculate the approximate focal length based on the calibration
        // This is a simplified approach - in a production app, you would use proper camera calibration
        let approximateFocalLength = pixelsPerMeter * 0.5 // Assuming the object is about 0.5m from the camera
        
        // Update the intrinsics with our calculated focal length
        let updatedIntrinsics = CameraIntrinsics(
            focalLengthX: approximateFocalLength,
            focalLengthY: approximateFocalLength,
            principalPointX: Double(previewLayer.bounds.width / 2),
            principalPointY: Double(previewLayer.bounds.height / 2)
        )
        calibrator.setIntrinsics(updatedIntrinsics)
        
        // Determine if this is the first or second device
        let isFirstDevice = mcSession.connectedPeers.isEmpty || 
                           (mcSession.connectedPeers.first?.displayName ?? "") > UIDevice.current.name
        
        // Calculate the camera position based on the calibration object
        // For simplicity, we'll still use a fixed configuration, but with better scale
        if isFirstDevice {
            // First device is at origin
            calibrator.setExtrinsics(translation: [0, 0, 0], rotation: [0, 0, 0])
        } else {
            // Second device - we'll still assume it's to the right, but we'll use the calculated scale
            // In a real implementation, you would calculate the actual relative position
            calibrator.setExtrinsics(translation: [1.0, 0, 0], rotation: [0, 0, 0])
        }
        
        // Store the pixels per meter value for distance calculations
        UserDefaults.standard.set(pixelsPerMeter, forKey: "pixelsPerMeter")
        
        // Add the calibrator to the triangulator
        triangulator.addCalibrator(calibrator: calibrator, deviceId: deviceId)
        print("Added local calibrator to triangulator for device: \(deviceId)")
        print("Available calibrators: \(triangulator.getAvailableCalibrators())")
        
        // Share calibration data with peers
        shareCalibrationData(calibrator: calibrator, deviceId: deviceId, pixelsPerMeter: pixelsPerMeter)
    }
    
    private func shareCalibrationData(calibrator: CameraCalibrator, deviceId: String, pixelsPerMeter: Double) {
        guard !mcSession.connectedPeers.isEmpty else { return }
        
        do {
            // Convert calibration data to dictionary
            let intrinsicsDict = calibrator.intrinsics.toDictionary()
            let extrinsicsDict = calibrator.extrinsics.toDictionary()
            
            let calibrationData: [String: Any] = [
                "type": "calibration",
                "device_id": deviceId,
                "intrinsics": intrinsicsDict,
                "extrinsics": extrinsicsDict,
                "pixels_per_meter": pixelsPerMeter,
                "object_width_meters": calibrationObjectWidthInches * inchesToMeters,
                "object_height_meters": calibrationObjectHeightInches * inchesToMeters
            ]
            
            print("Sharing calibration data for device ID: \(deviceId)")
            
            // Send to peers
            let data = try JSONSerialization.data(withJSONObject: calibrationData, options: [])
            try mcSession.send(data, toPeers: mcSession.connectedPeers, with: .reliable)
            
            print("Calibration data sent to \(mcSession.connectedPeers.count) peers")
        } catch {
            print("Error sending calibration data: \(error.localizedDescription)")
        }
    }
    
    // MARK: - Triangulation Methods
    
    private func sendDetectionToPeers(detection: Detection2D) {
        guard !mcSession.connectedPeers.isEmpty else { return }
        
        do {
            // Make sure we're using the correct device ID
            var detectionToSend = detection
            // If the detection doesn't have the correct device ID, create a new one with the correct ID
            if detection.deviceId != UIDevice.current.identifierForVendor?.uuidString ?? "unknown-device" {
                detectionToSend = Detection2D(
                    x: detection.x,
                    y: detection.y,
                    width: detection.width,
                    height: detection.height,
                    confidence: detection.confidence,
                    timestamp: detection.timestamp,
                    deviceId: UIDevice.current.identifierForVendor?.uuidString ?? "unknown-device"
                )
            }
            
            let detectionDict = detectionToSend.toDictionary()
            let data = try JSONSerialization.data(withJSONObject: ["type": "detection", "data": detectionDict], options: [])
            try mcSession.send(data, toPeers: mcSession.connectedPeers, with: .reliable)
        } catch {
            print("Error sending detection data: \(error.localizedDescription)")
        }
    }
    
    private func updateTriangulationDisplay() {
        guard let position = triangulator.getLatestPosition() else {
            // If no position is available, update the label to show status
            DispatchQueue.main.async {
                self.triangulationLabel.text = "Waiting for triangulation data...\nNeed detections from both cameras"
                self.triangulationLabel.backgroundColor = UIColor.darkGray.withAlphaComponent(0.7)
            }
            return
        }
        
        // Calculate time since last triangulation
        let timeSinceUpdate = CACurrentMediaTime() - lastTriangulationTime
        let freshness = timeSinceUpdate < 0.5 ? "Fresh" : "Stale"
        
        // Update UI on main thread
        DispatchQueue.main.async {
            // Format the position with 2 decimal places
            let positionText = String(format: "Ball Position (m):\nX: %.2f Y: %.2f Z: %.2f\nConfidence: %.2f (%@)",
                                     position.x, position.y, position.z, position.confidence, freshness)
            
            self.triangulationLabel.text = positionText
            
            // Change background color based on confidence
            if position.confidence > 0.7 {
                self.triangulationLabel.backgroundColor = UIColor.green.withAlphaComponent(0.7)
            } else if position.confidence > 0.4 {
                self.triangulationLabel.backgroundColor = UIColor.orange.withAlphaComponent(0.7)
            } else {
                self.triangulationLabel.backgroundColor = UIColor.red.withAlphaComponent(0.7)
            }
            
            // Make sure the label is visible
            self.triangulationLabel.isHidden = false
            
            // Bring the label to front
            self.view.bringSubviewToFront(self.triangulationLabel)
        }
    }
}
