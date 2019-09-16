
"use strict";

let IntersectionPose = require('./IntersectionPose.js');
let ObstacleProjectedDetectionList = require('./ObstacleProjectedDetectionList.js');
let SourceTargetNodes = require('./SourceTargetNodes.js');
let AprilTagExtended = require('./AprilTagExtended.js');
let CoordinationClearance = require('./CoordinationClearance.js');
let Vsample = require('./Vsample.js');
let LEDInterpreter = require('./LEDInterpreter.js');
let TagInfo = require('./TagInfo.js');
let AntiInstagramTransform = require('./AntiInstagramTransform.js');
let VehicleCorners = require('./VehicleCorners.js');
let LanePose = require('./LanePose.js');
let FSMState = require('./FSMState.js');
let ObstacleType = require('./ObstacleType.js');
let Vector2D = require('./Vector2D.js');
let SceneSegments = require('./SceneSegments.js');
let SignalsDetectionETHZ17 = require('./SignalsDetectionETHZ17.js');
let MaintenanceState = require('./MaintenanceState.js');
let LEDDetectionArray = require('./LEDDetectionArray.js');
let ObstacleImageDetectionList = require('./ObstacleImageDetectionList.js');
let DuckieSensor = require('./DuckieSensor.js');
let LightSensor = require('./LightSensor.js');
let AntiInstagramHealth = require('./AntiInstagramHealth.js');
let AprilTagsWithInfos = require('./AprilTagsWithInfos.js');
let IntersectionPoseImgDebug = require('./IntersectionPoseImgDebug.js');
let Trajectory = require('./Trajectory.js');
let LEDDetectionDebugInfo = require('./LEDDetectionDebugInfo.js');
let CarControl = require('./CarControl.js');
let ThetaDotSample = require('./ThetaDotSample.js');
let VehiclePose = require('./VehiclePose.js');
let LEDPattern = require('./LEDPattern.js');
let LEDDetection = require('./LEDDetection.js');
let WheelsCmd = require('./WheelsCmd.js');
let BoolStamped = require('./BoolStamped.js');
let AprilTagDetectionArray = require('./AprilTagDetectionArray.js');
let Segment = require('./Segment.js');
let IntersectionPoseImg = require('./IntersectionPoseImg.js');
let ObstacleImageDetection = require('./ObstacleImageDetection.js');
let CoordinationSignal = require('./CoordinationSignal.js');
let Twist2DStamped = require('./Twist2DStamped.js');
let StopLineReading = require('./StopLineReading.js');
let DuckiebotLED = require('./DuckiebotLED.js');
let StreetNameDetection = require('./StreetNameDetection.js');
let Rect = require('./Rect.js');
let KinematicsWeights = require('./KinematicsWeights.js');
let SegmentList = require('./SegmentList.js');
let AprilTagDetection = require('./AprilTagDetection.js');
let Rects = require('./Rects.js');
let Pixel = require('./Pixel.js');
let WheelsCmdStamped = require('./WheelsCmdStamped.js');
let StreetNames = require('./StreetNames.js');
let TurnIDandType = require('./TurnIDandType.js');
let Pose2DStamped = require('./Pose2DStamped.js');
let ObstacleProjectedDetection = require('./ObstacleProjectedDetection.js');
let SignalsDetection = require('./SignalsDetection.js');
let KinematicsParameters = require('./KinematicsParameters.js');
let AntiInstagramTransform_CB = require('./AntiInstagramTransform_CB.js');

module.exports = {
  IntersectionPose: IntersectionPose,
  ObstacleProjectedDetectionList: ObstacleProjectedDetectionList,
  SourceTargetNodes: SourceTargetNodes,
  AprilTagExtended: AprilTagExtended,
  CoordinationClearance: CoordinationClearance,
  Vsample: Vsample,
  LEDInterpreter: LEDInterpreter,
  TagInfo: TagInfo,
  AntiInstagramTransform: AntiInstagramTransform,
  VehicleCorners: VehicleCorners,
  LanePose: LanePose,
  FSMState: FSMState,
  ObstacleType: ObstacleType,
  Vector2D: Vector2D,
  SceneSegments: SceneSegments,
  SignalsDetectionETHZ17: SignalsDetectionETHZ17,
  MaintenanceState: MaintenanceState,
  LEDDetectionArray: LEDDetectionArray,
  ObstacleImageDetectionList: ObstacleImageDetectionList,
  DuckieSensor: DuckieSensor,
  LightSensor: LightSensor,
  AntiInstagramHealth: AntiInstagramHealth,
  AprilTagsWithInfos: AprilTagsWithInfos,
  IntersectionPoseImgDebug: IntersectionPoseImgDebug,
  Trajectory: Trajectory,
  LEDDetectionDebugInfo: LEDDetectionDebugInfo,
  CarControl: CarControl,
  ThetaDotSample: ThetaDotSample,
  VehiclePose: VehiclePose,
  LEDPattern: LEDPattern,
  LEDDetection: LEDDetection,
  WheelsCmd: WheelsCmd,
  BoolStamped: BoolStamped,
  AprilTagDetectionArray: AprilTagDetectionArray,
  Segment: Segment,
  IntersectionPoseImg: IntersectionPoseImg,
  ObstacleImageDetection: ObstacleImageDetection,
  CoordinationSignal: CoordinationSignal,
  Twist2DStamped: Twist2DStamped,
  StopLineReading: StopLineReading,
  DuckiebotLED: DuckiebotLED,
  StreetNameDetection: StreetNameDetection,
  Rect: Rect,
  KinematicsWeights: KinematicsWeights,
  SegmentList: SegmentList,
  AprilTagDetection: AprilTagDetection,
  Rects: Rects,
  Pixel: Pixel,
  WheelsCmdStamped: WheelsCmdStamped,
  StreetNames: StreetNames,
  TurnIDandType: TurnIDandType,
  Pose2DStamped: Pose2DStamped,
  ObstacleProjectedDetection: ObstacleProjectedDetection,
  SignalsDetection: SignalsDetection,
  KinematicsParameters: KinematicsParameters,
  AntiInstagramTransform_CB: AntiInstagramTransform_CB,
};
