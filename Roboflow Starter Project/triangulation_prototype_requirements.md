# 3D Ball Position Triangulation Prototype
## Product Requirements Document

## 1. Overview

This document outlines the requirements for developing a prototype system that uses two iOS devices (iPhone 12 Pro and iPad Pro 5th gen 12.9") to triangulate the 3D position of a pickleball in space. The system will leverage the existing object detection capabilities and MultipeerConnectivity (MCP) framework to share detection data between devices in real-time.

## 2. Background

The current application can detect objects (including balls) using computer vision and share this detection data between devices. However, it lacks the capability to combine data from multiple viewpoints to determine the 3D position of objects. This prototype aims to extend the functionality to enable 3D position triangulation.

## 3. Goals

- Create a calibration process to establish a common coordinate system between two iOS devices
- Implement triangulation algorithms to determine the 3D position of a pickleball from two camera viewpoints
- Visualize the ball's position in 3D space relative to a fixed reference point
- Achieve real-time performance with minimal latency (target: 30Hz update rate)
- Maintain accuracy within 5-10cm in a typical pickleball court environment

## 4. Non-Goals

- Tracking multiple balls simultaneously
- Tracking players or other objects
- Production-ready UI/UX
- Integration with scoring or game analysis features
- Support for more than two devices

## 5. System Architecture

### 5.1 Hardware Components

- iPhone 12 Pro (Device A)
  - Camera: 12MP, f/1.6 aperture
  - Field of view: ~65 degrees (wide camera)
  - Focal length: ~26mm (35mm equivalent)

- iPad Pro 5th gen 12.9" (Device B)
  - Camera: 12MP, f/1.8 aperture
  - Field of view: ~60 degrees (wide camera)
  - Focal length: ~28mm (35mm equivalent)

- Pickleball
  - Diameter: 74mm (2.9 inches)
  - Color: High-visibility yellow (for optimal detection)

- Calibration Object
  - A fixed-position object with known dimensions
  - Recommended: Cube or checkerboard pattern with minimum 20cm sides

### 5.2 Software Components

1. **Object Detection Module**
   - Use existing Roboflow model, optimized for pickleball detection
   - Filter to detect only the ball, ignoring court lines and other objects

2. **Camera Calibration Module**
   - Determine intrinsic parameters (focal length, principal point, distortion)
   - Calculate extrinsic parameters (position and orientation) relative to calibration object

3. **Data Sharing Module**
   - Extend existing MCP implementation to share:
     - Calibration parameters
     - Ball detection coordinates
     - Timestamp for synchronization

4. **Triangulation Module**
   - Implement algorithms to calculate 3D position from two 2D projections
   - Apply filtering to reduce noise and jitter

5. **Visualization Module**
   - Display 3D position in a common coordinate system
   - Show error metrics and confidence values

## 6. Detailed Requirements

### 6.1 Calibration Process

1. **Device Setup**
   - Devices should be positioned with clear, overlapping views of the play area
   - Recommended: 45-90 degree angle between camera viewpoints
   - Stable mounting required (tripods or fixed stands)

2. **Intrinsic Calibration**
   - Pre-calibrate each device to determine:
     - Focal length in pixels
     - Principal point (optical center)
     - Lens distortion parameters
   - Store these parameters for use in triangulation

3. **Extrinsic Calibration**
   - Place calibration object in a location visible to both cameras
   - Detect the calibration object in both camera views
   - Calculate the relative position and orientation of each camera
   - Establish a common world coordinate system

4. **Coordinate System**
   - Origin: Center of calibration object
   - X-axis: Horizontal (parallel to court)
   - Y-axis: Vertical (perpendicular to court)
   - Z-axis: Depth (following right-hand rule)
   - Units: Meters

5. **Calibration Verification**
   - Place the ball at known positions to verify accuracy
   - Recalibrate if error exceeds 10cm

### 6.2 Ball Detection

1. **Model Requirements**
   - Use existing Roboflow model trained on pickleball images
   - Detection confidence threshold: 0.7
   - Update rate: Minimum 30Hz

2. **Detection Data**
   - For each detection, extract:
     - Center point (x, y) in image coordinates
     - Bounding box dimensions (for size estimation)
     - Confidence score
     - Timestamp

3. **Size-Based Distance Estimation**
   - Use known ball size (74mm) and apparent size in image to estimate distance
   - Serve as a validation check for triangulation results

### 6.3 Data Transmission

1. **Packet Structure**
   - Extend current packet format to include:
     - Device identifier
     - Camera parameters (if changed)
     - Ball detection coordinates (normalized)
     - Timestamp (high precision)

2. **Performance Requirements**
   - Transmission rate: 60Hz (matching current implementation)
   - Maximum latency: 50ms
   - Packet size: <1KB

3. **Synchronization**
   - Implement clock synchronization between devices
   - Match detections based on timestamps
   - Discard detections older than 100ms

### 6.4 Triangulation Algorithm

1. **Method**
   - Implement linear triangulation using Direct Linear Transform (DLT)
   - Apply RANSAC for outlier rejection
   - Use Kalman filtering for trajectory smoothing

2. **Input**
   - 2D coordinates from both cameras
   - Camera intrinsic and extrinsic parameters
   - Timestamps for synchronization

3. **Output**
   - 3D coordinates (x, y, z) in world coordinate system
   - Confidence/error estimate
   - Velocity vector (optional)

### 6.5 Visualization

1. **3D Position Display**
   - Show ball position in a 3D coordinate system
   - Indicate position relative to calibration object
   - Display height above ground and distance from each camera

2. **Trajectory Visualization**
   - Plot recent ball trajectory (last 1-2 seconds)
   - Color-code by height or velocity

3. **Debugging Information**
   - Display raw detection data from both cameras
   - Show triangulation error estimates
   - Visualize camera positions and orientations

## 7. User Interface

### 7.1 Calibration UI

1. **Setup Screen**
   - Instructions for positioning devices
   - Camera preview with calibration object overlay
   - Progress indicators for calibration steps

2. **Calibration Controls**
   - "Begin Calibration" button
   - "Verify Calibration" button
   - "Reset Calibration" button

### 7.2 Main UI

1. **Camera View**
   - Live camera feed with ball detection overlay
   - Toggle for debug information visibility

2. **3D Visualization View**
   - Top-down view of ball position
   - Side view showing height
   - Optional: 3D perspective view

3. **Status Information**
   - Connection status between devices
   - Calibration status
   - Detection confidence
   - Triangulation error estimate

## 8. Implementation Plan

### 8.1 Phase 1: Setup and Calibration

1. Modify object detection to focus only on pickleballs
2. Implement camera calibration module
3. Extend MCP to share calibration data
4. Create calibration UI

### 8.2 Phase 2: Triangulation

1. Implement triangulation algorithm
2. Add synchronization between device detections
3. Apply filtering and error estimation
4. Test with static ball positions

### 8.3 Phase 3: Visualization and Testing

1. Develop 3D visualization interface
2. Implement trajectory tracking
3. Test with moving ball scenarios
4. Optimize for performance and accuracy

## 9. Testing Methodology

### 9.1 Calibration Testing

1. Verify intrinsic parameters against manufacturer specifications
2. Test extrinsic calibration with objects at known positions
3. Measure calibration stability over time

### 9.2 Accuracy Testing

1. Place ball at known 3D coordinates
2. Compare triangulated position with ground truth
3. Test at various distances and angles
4. Measure precision (repeatability) and accuracy (correctness)

### 9.3 Performance Testing

1. Measure end-to-end latency from ball movement to position calculation
2. Test frame rate under various lighting conditions
3. Evaluate battery and thermal impact during extended use

## 10. Success Criteria

1. **Accuracy**: Position error <10cm at distances up to 5m
2. **Latency**: End-to-end processing time <100ms
3. **Robustness**: Reliable detection in various lighting conditions
4. **Usability**: Calibration process completable in <5 minutes

## 11. Future Considerations

1. Support for additional cameras to improve accuracy
2. Integration with game analysis features
3. Trajectory prediction capabilities
4. Automatic recalibration during use
5. Extension to track multiple objects simultaneously

## 12. Questions and Assumptions

1. **Assumption**: Devices will have a clear line of sight to the ball at all times
2. **Assumption**: Lighting conditions will be consistent and adequate
3. **Question**: Will the system be used indoors, outdoors, or both?
4. **Question**: Is there a specific accuracy requirement for the prototype?
5. **Question**: Are there constraints on the positioning of the devices?

## 13. Appendix

### 13.1 Triangulation Mathematics

The basic triangulation equation using the Direct Linear Transform (DLT) method:

For each camera i, we have:
λᵢ * xᵢ = P_i * X

Where:
- λᵢ is a scale factor
- xᵢ is the 2D point in camera i's image plane
- P_i is the projection matrix for camera i
- X is the 3D point we're solving for

This can be rearranged into a system of linear equations and solved using SVD.

### 13.2 Calibration Object Specifications

The calibration object should be:
- Rigid and stable
- Have clearly defined corners or features
- Known dimensions (measured to within 1mm)
- Non-reflective surface
- Visible from multiple angles

A checkerboard pattern (8x8 squares, 25mm per square) is recommended.

### 13.3 References

1. Hartley, R., & Zisserman, A. (2003). Multiple View Geometry in Computer Vision. Cambridge University Press.
2. Apple Developer Documentation: AVFoundation and ARKit
3. OpenCV Camera Calibration Documentation 