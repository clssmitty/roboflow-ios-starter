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


class ViewController: UIViewController, AVCaptureVideoDataOutputSampleBufferDelegate, MCSessionDelegate, MCNearbyServiceAdvertiserDelegate, MCNearbyServiceBrowserDelegate {
    
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
    
    @IBOutlet weak var connectionStatusLabel: UILabel!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view. //roboflow-mask-wearing-ios
        
        loadRoboflowModelWith(model: "line-detector-yytrt", version: 1 , threshold: 0.7, overlap: 0.2, maxObjects: 100.0)
        checkCameraAuthorization()
        
        // Setup MultipeerConnectivity
        setupMultipeerConnectivity()
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        // Clean up MultipeerConnectivity
        autoSendTimer?.invalidate()
        autoSendTimer = nil
        
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
            
            // If you detect lines vs. balls:
            // handle them differently as you wish:
            if detectedClass == "ball" {
                // We'll just track the first "pickleball"
                // If you want multiple, store them in a dictionary keyed by ID or do matching.
                let cX = boundingBox.midX
                let cY = boundingBox.midY
                ballCenter = CGPoint(x: cX, y: cY)
            }
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
        
        // Add connection buttons
        addConnectionButtons()
    }
    
    func addConnectionButtons() {
        // Host button
        let hostButton = UIButton(frame: CGRect(x: 20, y: view.bounds.height - 150, width: view.bounds.width/2 - 30, height: 50))
        hostButton.setTitle("Host", for: .normal)
        hostButton.backgroundColor = .systemBlue
        hostButton.layer.cornerRadius = 8
        hostButton.addTarget(self, action: #selector(startHosting), for: .touchUpInside)
        view.addSubview(hostButton)
        
        // Join button
        let joinButton = UIButton(frame: CGRect(x: view.bounds.width/2 + 10, y: view.bounds.height - 150, width: view.bounds.width/2 - 30, height: 50))
        joinButton.setTitle("Join", for: .normal)
        joinButton.backgroundColor = .systemGreen
        joinButton.layer.cornerRadius = 8
        joinButton.addTarget(self, action: #selector(joinSession), for: .touchUpInside)
        view.addSubview(joinButton)
        
        // Send Data button
        let sendButton = UIButton(frame: CGRect(x: 20, y: view.bounds.height - 90, width: view.bounds.width - 40, height: 50))
        sendButton.setTitle("Send Detection Data", for: .normal)
        sendButton.backgroundColor = .systemOrange
        sendButton.layer.cornerRadius = 8
        sendButton.addTarget(self, action: #selector(sendDetectionData), for: .touchUpInside)
        view.addSubview(sendButton)
        
        // Auto-send toggle button
        let autoSendButton = UIButton(frame: CGRect(x: 20, y: view.bounds.height - 30, width: view.bounds.width - 40, height: 30))
        autoSendButton.setTitle("Toggle Auto-Send", for: .normal)
        autoSendButton.backgroundColor = .systemPurple
        autoSendButton.layer.cornerRadius = 8
        autoSendButton.addTarget(self, action: #selector(toggleAutoSend), for: .touchUpInside)
        view.addSubview(autoSendButton)
    }
    
    @objc func startHosting() {
        mcAdvertiser.startAdvertisingPeer()
        updateConnectionStatus(status: "Hosting - Waiting for connections")
    }
    
    @objc func joinSession() {
        // Present browser view controller
        let mcBrowserVC = MCBrowserViewController(serviceType: serviceType, session: mcSession)
        mcBrowserVC.delegate = self
        present(mcBrowserVC, animated: true)
    }
    
    @objc func sendDetectionData() {
        // Only send if we have peers connected
        guard !mcSession.connectedPeers.isEmpty else {
            showAlert(title: "No Connections", message: "No peers connected to send data to.")
            return
        }
        
        // Create a simple message with the latest detection data
        if let detections = getLatestDetections() {
            do {
                let data = try JSONSerialization.data(withJSONObject: detections, options: [])
                try mcSession.send(data, toPeers: mcSession.connectedPeers, with: .reliable)
                print("Sent detection data to \(mcSession.connectedPeers.count) peers")
            } catch {
                print("Error sending data: \(error.localizedDescription)")
            }
        }
    }
    
    @objc func toggleAutoSend() {
        if autoSendTimer == nil {
            // Start auto-sending
            autoSendTimer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
                self?.sendDetectionData()
            }
            updateConnectionStatus(status: "Auto-send enabled")
        } else {
            // Stop auto-sending
            autoSendTimer?.invalidate()
            autoSendTimer = nil
            updateConnectionStatus(status: "Auto-send disabled")
        }
    }
    
    func getLatestDetections() -> [[String: Any]]? {
        // Convert the latest detection results to a serializable format
        var serializableDetections: [[String: Any]] = []
        
        for detection in latestDetectionResults {
            if let detectionInfo = detection.vals() as? [String: Any] {
                serializableDetections.append(detectionInfo)
            }
        }
        
        // If we have no detections, return nil or sample data
        if serializableDetections.isEmpty {
            // Return nil or sample data for testing
            return [
                ["class": "sample", "confidence": 0.5, "x": 100, "y": 100, "width": 50, "height": 50]
            ]
        }
        
        return serializableDetections
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
        case .connecting:
            print("Connecting to: \(peerID.displayName)")
            updateConnectionStatus(status: "Connecting to: \(peerID.displayName)")
        case .notConnected:
            print("Not connected to: \(peerID.displayName)")
            updateConnectionStatus(status: "Disconnected from: \(peerID.displayName)")
        @unknown default:
            print("Unknown connection state: \(peerID.displayName)")
        }
    }
    
    func session(_ session: MCSession, didReceive data: Data, fromPeer peerID: MCPeerID) {
        // Process received data
        do {
            if let receivedDetections = try JSONSerialization.jsonObject(with: data) as? [[String: Any]] {
                print("Received detection data from \(peerID.displayName):")
                for detection in receivedDetections {
                    print(detection)
                }
                
                // Store the received detections
                self.receivedDetections = receivedDetections
                
                // Here you could process the received detections
                // For example, display them on screen or combine with local detections
                DispatchQueue.main.async {
                    // Update the UI to show we received data
                    self.updateConnectionStatus(status: "Received data from: \(peerID.displayName)")
                    
                    // Redraw the detection overlay to include received detections
                    self.drawReceivedDetections()
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
}

// MARK: - MCBrowserViewControllerDelegate
extension ViewController: MCBrowserViewControllerDelegate {
    func browserViewControllerDidFinish(_ browserViewController: MCBrowserViewController) {
        dismiss(animated: true)
        updateConnectionStatus(status: "Connected to \(mcSession.connectedPeers.count) peers")
    }
    
    func browserViewControllerWasCancelled(_ browserViewController: MCBrowserViewController) {
        dismiss(animated: true)
        updateConnectionStatus(status: "Browsing cancelled")
    }
}

// Draw detections received from peers with a different color
func drawReceivedDetections() {
    // Use a different color for received detections to distinguish them
    let peerColor = UIColor(red: 1.0, green: 0.5, blue: 0.0, alpha: 0.4) // Orange
    
    for detection in receivedDetections {
        guard
            let detectedClass = detection["class"] as? String,
            let confidence = detection["confidence"] as? Double,
            let x = detection["x"] as? Float,
            let y = detection["y"] as? Float,
            let width = detection["width"] as? Float,
            let height = detection["height"] as? Float
        else {
            continue
        }
        
        let bounds = detectionOverlay.bounds
        let xs = bounds.width / bufferSize.width
        let ys = bounds.height / bufferSize.height
        
        let boundingBox = CGRect(
            x: CGFloat(x) * xs,
            y: CGFloat(y) * ys,
            width: CGFloat(width) * xs,
            height: CGFloat(height) * ys
        )
        
        // Draw the bounding box with a special label to indicate it's from a peer
        drawBoundingBox(boundingBox: boundingBox,
                        color: peerColor,
                        detectedValue: "\(detectedClass) (PEER)",
                        confidence: confidence)
    }
}
